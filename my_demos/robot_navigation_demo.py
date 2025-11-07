#!/usr/bin/env python3
"""
工作的视频生成器 - 基于成功的仿真
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# 添加项目路径
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from nav_world.nav_env import NavEnv

def main():
    print("🎬 工作视频生成器")
    print("=" * 40)
    
    # 创建环境
    xml_path = os.path.join(project_root, "nav_world", "room.xml")
    env = NavEnv(xml_path=xml_path, grid_res=0.1)
    env.reset()
    
    print("✅ 环境初始化完成")
    print(f"   Alice: {env._get_body_xy('alice')}")
    print(f"   Bob: {env._get_body_xy('bob')}")
    
    # 运行仿真并记录轨迹
    print("\n🚀 开始仿真...")
    T = 15.0  # 增加仿真时间到15秒
    fps = 30
    dt = 1.0 / fps
    steps = int(T * fps)
    
    alice_positions = []
    bob_positions = []
    
    for step in range(steps):
        obs, done = env.step(dt=dt)
        
        alice_pos = env._get_body_xy('alice')
        bob_pos = env._get_body_xy('bob')
        alice_positions.append(alice_pos)
        bob_positions.append(bob_pos)
        
        if step % 30 == 0:
            print(f"   步骤 {step}: Alice{alice_pos}, Bob{bob_pos}")
        
        if done:
            print(f"✅ 任务完成！用时 {step/fps:.1f}秒")
            break
    
    # 创建动画视频
    print(f"\n🎬 创建动画视频 ({len(alice_positions)}帧)...")
    
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # 设置房间边界
    ax.set_xlim(-4, 4)
    ax.set_ylim(-3, 3)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_title('双机器人导航任务动画')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    
    # 绘制障碍物 - 匹配新的room.xml布局
    obstacles = [
        {'pos': (-2.0, 1.6), 'size': (0.3, 1.1), 'color': 'orange'},  # obst_top_left
        {'pos': (2.0, 2.0), 'size': (0.3, 1.2), 'color': 'orange'},   # obst_top_right
        {'pos': (0.5, 0.0), 'size': (0.2, 1.0), 'color': 'orange'},   # obst_midcol
        {'pos': (-1.5, -0.5), 'size': (1.0, 0.2), 'color': 'orange'}, # obst_left_bar
        {'pos': (1.5, -1.0), 'size': (1.0, 0.2), 'color': 'orange'},  # obst_right_bar
        {'pos': (0.0, -2.2), 'size': (0.8, 0.2), 'color': 'orange'},  # obst_bottom
    ]
    
    for obs in obstacles:
        rect = plt.Rectangle(
            (obs['pos'][0] - obs['size'][0], obs['pos'][1] - obs['size'][1]),
            2 * obs['size'][0], 2 * obs['size'][1],
            facecolor=obs['color'], alpha=0.3
        )
        ax.add_patch(rect)
    
    # 绘制目标点 - 匹配新的room.xml布局
    ax.scatter(3.0, 1.6, color='blue', s=200, marker='*', label='Alice目标')
    ax.scatter(3.2, -1.0, color='green', s=200, marker='*', label='Bob目标')
    
    # 初始化轨迹线
    alice_line, = ax.plot([], [], 'b-', linewidth=2, label='Alice轨迹')
    bob_line, = ax.plot([], [], 'r-', linewidth=2, label='Bob轨迹')
    
    # 初始化机器人点
    alice_point, = ax.plot([], [], 'bo', markersize=10, label='Alice')
    bob_point, = ax.plot([], [], 'ro', markersize=10, label='Bob')
    
    ax.legend()
    
    def animate(frame):
        if frame < len(alice_positions):
            # 更新轨迹
            alice_line.set_data([p[0] for p in alice_positions[:frame+1]], 
                               [p[1] for p in alice_positions[:frame+1]])
            bob_line.set_data([p[0] for p in bob_positions[:frame+1]], 
                             [p[1] for p in bob_positions[:frame+1]])
            
            # 更新当前位置
            alice_point.set_data([alice_positions[frame][0]], [alice_positions[frame][1]])
            bob_point.set_data([bob_positions[frame][0]], [bob_positions[frame][1]])
        
        return alice_line, bob_line, alice_point, bob_point
    
    # 创建动画
    anim = animation.FuncAnimation(fig, animate, frames=len(alice_positions), 
                                  interval=1000/fps, blit=True, repeat=True)
    
    # 保存为MP4
    try:
        anim.save('robot_navigation_animation.mp4', writer='ffmpeg', fps=fps)
        print("✅ 动画视频保存: robot_navigation_animation.mp4")
    except Exception as e:
        print(f"⚠️ MP4保存失败: {e}")
        try:
            anim.save('robot_navigation_animation.gif', writer='pillow', fps=fps)
            print("✅ GIF动画保存: robot_navigation_animation.gif")
        except Exception as e2:
            print(f"⚠️ GIF保存也失败: {e2}")
    
    # 保存静态轨迹图
    plt.savefig('robot_navigation_trajectory.png', dpi=150)
    print("✅ 静态轨迹图保存: robot_navigation_trajectory.png")
    
    print(f"\n🎯 视频生成完成！")
    print(f"   总步数: {len(alice_positions)}")
    print(f"   总时间: {len(alice_positions)/fps:.1f}秒")
    print(f"   Alice最终位置: {alice_positions[-1]}")
    print(f"   Bob最终位置: {bob_positions[-1]}")

if __name__ == "__main__":
    main()
