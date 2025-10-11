#!/usr/bin/env python3
"""
修复的MuJoCo 3D Viewer - 解决闪退问题
"""

import os
import sys
import time
import numpy as np

# 添加项目路径
project_root = os.path.dirname(os.path.abspath(__file__))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from real_world.nav_world.nav_env import NavEnv

def main():
    print("🎮 修复的MuJoCo 3D Viewer")
    print("=" * 40)
    
    # 创建环境
    xml_path = os.path.join(project_root, "real_world", "nav_world", "room.xml")
    env = NavEnv(xml_path=xml_path, grid_res=0.1)
    env.reset()
    
    print("✅ 环境初始化完成")
    print(f"   Alice: {env._get_body_xy('alice')}")
    print(f"   Bob: {env._get_body_xy('bob')}")
    
    try:
        import mujoco.viewer
        
        print("\n🎬 启动MuJoCo 3D Viewer...")
        print("   按ESC退出，按空格暂停/继续")
        
        # 使用最简单的viewer方法
        with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
            print("✅ 3D Viewer已启动！")
            print("   现在可以看到3D场景中的机器人")
            print("   等待2秒让viewer完全加载...")
            
            # 等待viewer完全加载
            time.sleep(2)
            
            # 检查viewer是否还在运行
            if not viewer.is_running():
                print("❌ Viewer已关闭")
                return
            
            print("🚀 开始运行仿真...")
            
            # 运行仿真
            T = 15.0  # 增加仿真时间到15秒
            fps = 30
            dt = 1.0 / fps
            steps = int(T * fps)
            
            for step in range(steps):
                # 检查viewer是否还在运行
                if not viewer.is_running():
                    print("👋 Viewer被用户关闭")
                    break
                
                # 执行一步仿真
                obs, done = env.step(dt=dt)
                
                # 更新viewer
                viewer.sync()
                
                # 打印进度
                if step % 30 == 0:
                    alice_pos = env._get_body_xy('alice')
                    bob_pos = env._get_body_xy('bob')
                    print(f"   步骤 {step}/{steps}: Alice{alice_pos}, Bob{bob_pos}")
                
                if done:
                    print(f"✅ 任务完成！用时 {step/fps:.1f}秒")
                    break
                    
                # 添加小延迟，让viewer有时间渲染
                time.sleep(0.01)
                    
        print("🎉 3D Viewer演示完成！")
        
    except Exception as e:
        print(f"❌ 3D Viewer启动失败: {e}")
        print("💡 尝试使用离屏渲染方法...")
        
        # 方法2: 离屏渲染（备用方案）
        try:
            print("\n🎬 使用离屏渲染生成视频...")
            
            frames = []
            T = 15.0  # 增加仿真时间到15秒
            fps = 30
            dt = 1.0 / fps
            steps = int(T * fps)
            
            for step in range(steps):
                obs, done = env.step(dt=dt)
                
                # 渲染一帧
                frame = env.render_rgb()
                frames.append(frame)
                
                if step % 30 == 0:
                    alice_pos = env._get_body_xy('alice')
                    bob_pos = env._get_body_xy('bob')
                    print(f"   步骤 {step}/{steps}: Alice{alice_pos}, Bob{bob_pos}")
                
                if done:
                    print(f"✅ 任务完成！用时 {step/fps:.1f}秒")
                    break
            
            # 保存视频
            if frames:
                import imageio.v2 as imageio
                video_path = os.path.join(project_root, "mujoco_3d_video.mp4")
                imageio.mimsave(video_path, frames, fps=fps)
                print(f"✅ 3D视频已保存: {video_path}")
            
        except Exception as e2:
            print(f"❌ 离屏渲染也失败: {e2}")
            print("💡 请检查MuJoCo安装和依赖")

if __name__ == "__main__":
    main()
