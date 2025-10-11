# real_world/nav_world/nav_env.py
from __future__ import annotations
import os
import math
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict

import numpy as np
import mujoco
from mujoco import MjModel, MjData

# ========= 工具：简单 A* =========
def astar(grid: np.ndarray, start: Tuple[int,int], goal: Tuple[int,int]) -> Optional[List[Tuple[int,int]]]:
    """
    grid: 0 可走, 1 障碍
    start/goal: (ix, iy)
    """
    h, w = grid.shape
    def inb(p): return 0 <= p[0] < h and 0 <= p[1] < w
    def neigh(p):
        for dx,dy in [(1,0),(-1,0),(0,1),(0,-1)]:
            q = (p[0]+dx, p[1]+dy)
            if inb(q) and grid[q[0], q[1]] == 0:
                yield q

    import heapq
    g = {start: 0}
    came = {}
    pq = [(abs(start[0]-goal[0]) + abs(start[1]-goal[1]), start)]
    visited = set()
    while pq:
        _, u = heapq.heappop(pq)
        if u in visited: 
            continue
        visited.add(u)
        if u == goal:
            # 回溯
            path = [u]
            while u in came:
                u = came[u]
                path.append(u)
            return path[::-1]
        for v in neigh(u):
            alt = g[u] + 1
            if v not in g or alt < g[v]:
                g[v] = alt
                came[v] = u
                f = alt + abs(v[0]-goal[0]) + abs(v[1]-goal[1])
                heapq.heappush(pq, (f, v))
    return None

@dataclass
class AgentState:
    name: str
    radius: float = 0.12  # 和 XML 几何体一致
    speed: float = 0.8    # m/s
    path_world: Optional[List[Tuple[float,float]]] = None
    path_ptr: int = 0

class NavEnv:
    """
    双机器人导航任务（带障碍）
    - 机器人：alice, bob（自由体）
    - 目标：goal_a, goal_b（两个 site）
    - 控制：纯运动学，沿预先 A* 规划的路径前进
    """
    def __init__(self, xml_path: str, grid_res: float = 0.1, render_w: int = 800, render_h: int = 600):
        self.xml_path = xml_path
        self.model: MjModel = mujoco.MjModel.from_xml_path(xml_path)
        self.data: MjData = mujoco.MjData(self.model)
        self.grid_res = grid_res
        self.render_w, self.render_h = render_w, render_h

        # 解析机器人与目标
        self.agent_names = ["alice", "bob"]
        self.goal_sites = {"alice": "goal_a", "bob": "goal_b"}

        # 推断场地边界（按内墙尺寸来）
        # 这里直接硬编码与 XML 一致：x ∈ [-4, 4], y ∈ [-3, 3]
        self.xmin, self.xmax = -4.0, 4.0
        self.ymin, self.ymax = -3.0, 3.0

        # 构建占据栅格
        self.grid, self.grid_xs, self.grid_ys = self._build_occupancy()

        # 内部状态
        self.agents: Dict[str, AgentState] = {n: AgentState(n) for n in self.agent_names}
        self._init_qpos_idx()

        # 预分配像素缓冲区（可截图/存视频）
        self.rgb = np.zeros((self.render_h, self.render_w, 3), dtype=np.uint8)

    # ---------- MuJoCo 相关工具 ----------
    def _init_qpos_idx(self):
        """
        对自由体（freejoint）：
        - qpos 前 3 个是位置 (x,y,z)
        - 接着 4 个是四元数
        用名字定位 body id -> joint id -> qpos addr
        """
        self.qpos_addr = {}
        for name in self.agent_names:
            bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
            jid = self.model.body_jntadr[bid]
            addr = self.model.jnt_qposadr[jid]
            self.qpos_addr[name] = addr

    def _set_body_xy(self, name: str, x: float, y: float, z: float = 0.1):
        addr = self.qpos_addr[name]
        # 设置位置
        self.data.qpos[addr + 0] = x
        self.data.qpos[addr + 1] = y
        self.data.qpos[addr + 2] = z
        # 朝向保持默认（不改四元数）

    def _get_body_xy(self, name: str) -> Tuple[float,float]:
        addr = self.qpos_addr[name]
        return float(self.data.qpos[addr + 0]), float(self.data.qpos[addr + 1])

    def _get_site_xy(self, site_name: str) -> Tuple[float,float]:
        sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        pos = self.model.site_pos[sid]
        return float(pos[0]), float(pos[1])

    # ---------- 栅格构建 ----------
    def _build_occupancy(self):
        xs = np.arange(self.xmin, self.xmax + 1e-6, self.grid_res)
        ys = np.arange(self.ymin, self.ymax + 1e-6, self.grid_res)
        grid = np.zeros((len(xs), len(ys)), dtype=np.uint8)

        def world2grid(wx, wy):
            ix = int(round((wx - self.xmin) / self.grid_res))
            iy = int(round((wy - self.ymin) / self.grid_res))
            return ix, iy

        # 墙：把边界一圈设为障碍
        grid[0, :] = 1
        grid[-1, :] = 1
        grid[:, 0] = 1
        grid[:, -1] = 1

        # 解析 obst_* 盒子，按其 size 在栅格上打障碍
        for gname in self._geom_names(prefix="obst_"):
            gid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, gname)
            pos = self.model.geom_pos[gid]
            size = self.model.geom_size[gid]
            # box size = (sx, sy, sz)
            sx, sy = float(size[0]), float(size[1])
            xmin, xmax = pos[0]-sx, pos[0]+sx
            ymin, ymax = pos[1]-sy, pos[1]+sy

            ix_min, iy_min = world2grid(xmin, ymin)
            ix_max, iy_max = world2grid(xmax, ymax)
            ix_min, ix_max = max(ix_min,0), min(ix_max,grid.shape[0]-1)
            iy_min, iy_max = max(iy_min,0), min(iy_max,grid.shape[1]-1)
            grid[ix_min:ix_max+1, iy_min:iy_max+1] = 1

        return grid, xs, ys

    def _geom_names(self, prefix: str) -> List[str]:
        names = []
        for i in range(self.model.ngeom):
            n = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, i)
            if n is not None and n.startswith(prefix):
                names.append(n)
        return names

    # ---------- 规划 & 运行 ----------
    def reset(self, randomize: bool = False):
        mujoco.mj_resetData(self.model, self.data)

        # 初始化两个机器人位置（可选随机）
        alice_xy = (-2.5, -2.0)
        bob_xy   = (-2.5,  2.0)
        if randomize:
            alice_xy = (-2.8, np.random.uniform(-2.5, -1.5))
            bob_xy   = (-2.8, np.random.uniform( 1.5,  2.5))

        self._set_body_xy("alice", *alice_xy)
        self._set_body_xy("bob",   *bob_xy)

        # 计算各自目标
        self.goal_xy = {agt: self._get_site_xy(self.goal_sites[agt]) for agt in self.agent_names}

        # 为每个机器人规划路径（栅格 A*）
        for agt in self.agent_names:
            start = self._world2grid(self._get_body_xy(agt))
            goal  = self._world2grid(self.goal_xy[agt])
            path_g = astar(self.grid.copy(), start, goal)
            if path_g is None:
                raise RuntimeError(f"[{agt}] A* 找不到路径，从 {start} 到 {goal}")
            # 栅格路径 -> 世界坐标中心
            path_w = [self._grid2world(ix, iy) for (ix,iy) in path_g]
            self.agents[agt].path_world = path_w
            self.agents[agt].path_ptr = 0

        return self._obs()

    def step(self, dt: float = 0.02):
        """
        让每个机器人沿路径前进；防止两机器人碰撞（最简：保持安全距离）
        """
        # 简单防撞：如果距离过近，慢一点
        def safe_speed(a_xy, b_xy, base_spd):
            d = np.linalg.norm(np.array(a_xy) - np.array(b_xy))
            if d < 0.5:
                return base_spd * 0.4
            return base_spd

        # 移动每个机器人
        for name in self.agent_names:
            st = self.agents[name]
            if st.path_ptr >= len(st.path_world):
                continue
            cur = np.array(self._get_body_xy(name))
            tar = np.array(st.path_world[st.path_ptr])
            v   = st.speed
            # 如果另一个机器人很近，降速
            other = "alice" if name == "bob" else "bob"
            v = safe_speed(cur, np.array(self._get_body_xy(other)), v)

            dir = tar - cur
            dist = np.linalg.norm(dir)
            if dist < 1e-6:
                st.path_ptr += 1
            else:
                step = v * dt
                if step >= dist:
                    new_xy = tar
                    st.path_ptr += 1
                else:
                    new_xy = cur + dir / dist * step
                self._set_body_xy(name, float(new_xy[0]), float(new_xy[1]))

        # 前向仿真一小步（虽然我们是运动学移动，但调用 step 保持一致）
        mujoco.mj_step(self.model, self.data)

        done = self._is_done()
        return self._obs(), done

    # ---------- 渲染 ----------
    def render_rgb(self) -> np.ndarray:
        cam = mujoco.MjvCamera()
        opt = mujoco.MjvOption()
        scn = mujoco.MjvScene(self.model, maxgeom=2000)
        con = mujoco.MjrContext(self.model, mujoco.mjtFontScale.mjFONTSCALE_150)

        # 俯视相机
        cam.lookat[:] = np.array([0, 0, 0])
        cam.azimuth = 90
        cam.elevation = -90
        cam.distance = 9.0

        mujoco.mjv_updateScene(self.model, self.data, opt, None, cam, mujoco.mjtCatBit.mjCAT_ALL, scn)
        mujoco.mjr_render(mujoco.MjrRect(0, 0, self.render_w, self.render_h), scn, con)
        mujoco.mjr_readPixels(self.rgb, None, mujoco.MjrRect(0, 0, self.render_w, self.render_h), con)
        mujoco.mjr_freeContext(con)
        return self.rgb[..., ::-1].copy()   # BGR->RGB

    # ---------- 辅助 ----------
    def _obs(self):
        obs = {name: {"xy": self._get_body_xy(name), "goal": self.goal_xy[name]} for name in self.agent_names}
        return obs

    def _is_done(self) -> bool:
        ok = True
        for name in self.agent_names:
            xy = np.array(self._get_body_xy(name))
            gy = np.array(self.goal_xy[name])
            if np.linalg.norm(xy - gy) > 0.15:
                ok = False
                break
        return ok

    def _world2grid(self, xy: Tuple[float,float]) -> Tuple[int,int]:
        x, y = xy
        ix = int(round((x - self.xmin)/self.grid_res))
        iy = int(round((y - self.ymin)/self.grid_res))
        ix = np.clip(ix, 0, len(self.grid_xs)-1)
        iy = np.clip(iy, 0, len(self.grid_ys)-1)
        return int(ix), int(iy)

    def _grid2world(self, ix: int, iy: int) -> Tuple[float,float]:
        # 取每个 cell 的中心
        wx = self.xmin + ix * self.grid_res
        wy = self.ymin + iy * self.grid_res
        return float(wx), float(wy)

# 简单可运行 demo
if __name__ == "__main__":
    here = os.path.dirname(__file__)
    xml = os.path.join(here, "room.xml")
    env = NavEnv(xml_path=xml, grid_res=0.1)
    env.reset()

    T = 15.0
    fps = 30
    dt = 1.0 / fps
    frames = []
    steps = int(T * fps)

    for step_i in range(steps):  # ✅ 改这里
        obs, done = env.step(dt=dt)
        if (step_i % 2) == 0:     # ✅ 用 step_i 而不是 _
            frame = env.render_rgb()
            frames.append(frame)
        if done:
            break
    print("Done:", done, "frames:", len(frames))

