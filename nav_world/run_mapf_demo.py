#!/usr/bin/env python3
"""
MAPF路径规划演示脚本

这个脚本展示了如何使用新的多智能体路径规划（MAPF）功能
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt

# 添加项目路径
# 获取项目根目录（nav_world的父目录）
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from nav_world.nav_env_mapf import NavEnvMAPF


def demo_mapf_planning():
    """演示MAPF路径规划"""
    print("=" * 60)
    print("MAPF多智能体路径规划演示")
    print("=" * 60)
    
    # 获取XML文件路径
    here = os.path.dirname(__file__)
    xml_path = os.path.join(here, "room.xml")
    
    if not os.path.exists(xml_path):
        print(f"❌ XML文件未找到: {xml_path}")
        return
    
    # 创建带MAPF支持的环境
    print("\n📦 初始化NavEnv with MAPF...")
    env = NavEnvMAPF(
        xml_path=xml_path,
        grid_res=0.1,
        priority_order=['alice', 'bob']  # Alice优先规划
    )
    print(f"✅ 环境初始化完成")
    print(f"   网格大小: {env.grid.shape}")
    print(f"   智能体: {env.agent_names}")
    print(f"   优先级顺序: {env.priority_order}")
    
    # 使用MAPF规划重置环境
    print("\n🔄 使用MAPF规划重置环境...")
    obs = env.reset(use_mapf=True)
    print("✅ MAPF规划完成")
    
    # 显示规划结果
    mapf_paths = env.get_mapf_paths()
    if mapf_paths:
        print("\n📊 MAPF路径规划结果:")
        for agent_id, path in mapf_paths.items():
            print(f"   {agent_id}:")
            print(f"     路径长度: {len(path)} 步")
            print(f"     起点: {path[0][:2]} (t={path[0][2]})")
            print(f"     终点: {path[-1][:2]} (t={path[-1][2]})")
            print(f"     总时长: {path[-1][2]} 时间步")
    
    # 显示初始状态
    print("\n📍 初始状态:")
    for agent_name in env.agent_names:
        agent_obs = obs[agent_name]
        pos = agent_obs['xy']
        goal = agent_obs['goal']
        dist = np.linalg.norm(np.array(pos) - np.array(goal))
        print(f"   {agent_name}: 位置={pos}, 目标={goal}, 距离={dist:.2f}m")
    
    # 运行仿真
    print("\n🏃 运行仿真...")
    T = 15.0
    fps = 30
    dt = 1.0 / fps
    steps = int(T * fps)
    
    trajectory = {name: [] for name in env.agent_names}
    
    for step_i in range(steps):
        obs, done = env.step(dt=dt)
        
        # 记录轨迹
        for name in env.agent_names:
            trajectory[name].append(obs[name]['xy'])
        
        # 每30步打印一次进度
        if step_i % 30 == 0:
            alice_xy = obs['alice']['xy']
            bob_xy = obs['bob']['xy']
            alice_goal = obs['alice']['goal']
            bob_goal = obs['bob']['goal']
            
            alice_dist = np.linalg.norm(np.array(alice_xy) - np.array(alice_goal))
            bob_dist = np.linalg.norm(np.array(bob_xy) - np.array(bob_goal))
            
            print(f"   步骤 {step_i:3d}: Alice距离={alice_dist:.2f}m, Bob距离={bob_dist:.2f}m")
        
        if done:
            print(f"\n✅ 任务完成！在步骤 {step_i} 完成")
            break
    
    # 可视化轨迹
    print("\n🎨 生成可视化...")
    visualize_trajectories(env, trajectory)
    
    print("\n✅ 演示完成！")


def visualize_trajectories(env, trajectory):
    """可视化机器人轨迹"""
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # 绘制障碍物
    obstacles = [
        {'pos': (-2.0, 1.6), 'size': (0.3, 1.1), 'color': 'orange'},
        {'pos': (2.0, 2.0), 'size': (0.3, 1.2), 'color': 'orange'},
        {'pos': (0.5, 0.0), 'size': (0.2, 1.0), 'color': 'orange'},
        {'pos': (-1.5, -0.5), 'size': (1.0, 0.2), 'color': 'orange'},
        {'pos': (1.5, -1.0), 'size': (1.0, 0.2), 'color': 'orange'},
        {'pos': (0.0, -2.2), 'size': (0.8, 0.2), 'color': 'orange'},
    ]
    
    for obs in obstacles:
        rect = plt.Rectangle(
            (obs['pos'][0] - obs['size'][0], obs['pos'][1] - obs['size'][1]),
            2 * obs['size'][0], 2 * obs['size'][1],
            facecolor=obs['color'], alpha=0.3, edgecolor='black'
        )
        ax.add_patch(rect)
    
    # 绘制轨迹
    colors = {'alice': 'blue', 'bob': 'green'}
    for name in env.agent_names:
        traj = trajectory[name]
        if len(traj) > 0:
            xs = [pos[0] for pos in traj]
            ys = [pos[1] for pos in traj]
            ax.plot(xs, ys, color=colors[name], linewidth=2, alpha=0.7, 
                   label=f'{name.capitalize()} Trajectory')
            
            # 起点和终点
            ax.plot(xs[0], ys[0], 'o', color=colors[name], markersize=10, 
                   markeredgecolor='black', markeredgewidth=1.5)
            ax.plot(xs[-1], ys[-1], 's', color=colors[name], markersize=10,
                   markeredgecolor='black', markeredgewidth=1.5)
    
    # 绘制目标点
    for name in env.agent_names:
        goal = env.goal_xy[name]
        ax.scatter(goal[0], goal[1], color=colors[name], s=200, marker='*',
                  edgecolors='black', linewidths=1.5, label=f'{name.capitalize()} Goal')
    
    ax.set_xlim(-4, 4)
    ax.set_ylim(-3, 3)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_title('MAPF Multi-Agent Path Planning - Trajectories', fontsize=14)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.legend(loc='best')
    
    # 保存图片
    output_path = os.path.join(project_root, 'mapf_navigation_result.png')
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"✅ 可视化已保存: {output_path}")
    
    plt.close()


if __name__ == "__main__":
    try:
        demo_mapf_planning()
    except KeyboardInterrupt:
        print("\n⏹️ 用户中断")
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()

