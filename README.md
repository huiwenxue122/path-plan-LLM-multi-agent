# RoCo: Dialectic Multi-Robot Collaboration with Large Language Models
Codebase for paper: RoCo: Dialectic Multi-Robot Collaboration with Large Language Models

[Mandi Zhao](https://mandizhao.github.io), [Shreeya Jain](https://www.linkedin.com), [Shuran Song](https://www.cs.columbia.edu/~shurans/) 

[Arxiv](https://arxiv.org/abs/2307.04738) | [Project Website](https://project-roco.github.io) 

 
<img src="method.jpeg" alt="method" width="800"/>


## 🎯 我的演示项目

### 双机器人导航演示
我基于原项目创建了一个双机器人导航演示，包含：
- **`my_demos/robot_navigation_demo.py`** - 主要演示脚本（matplotlib动画）
- **`my_demos/robot_navigation_animation.gif`** - 机器人运动动画
- **`my_demos/robot_navigation_trajectory.png`** - 轨迹图
- **`fixed_mujoco_viewer.py`** - MuJoCo 3D Viewer版本

### 运行我的演示

#### 方法1: MuJoCo 3D Viewer（推荐）
```bash
cd /Users/claire/co-robot-pathfinding
mjpython fixed_mujoco_viewer.py
```
- ✅ 真正的3D可视化
- ✅ 机器人会正常移动
- ✅ 实时交互控制

#### 方法2: Matplotlib动画
```bash
cd /Users/claire/co-robot-pathfinding
python my_demos/robot_navigation_demo.py
```
- ✅ 生成GIF动画
- ✅ 跨平台兼容

### 查看结果
```bash
open my_demos/robot_navigation_animation.gif
open my_demos/robot_navigation_trajectory.png
```

---

## 原项目设置
### setup conda env and package install
```
conda create -n roco python=3.8 
conda activate roco
```
### Install mujoco and dm_control 
```
pip install mujoco==2.3.0
pip install dm_control==1.0.8 
```
**If you have M1 Macbook like me and would like to visualize the task scenes locally:**

Download the macos-compatible `.dmg` file from [MuJoCo release page](https://github.com/deepmind/mujoco/releases), inside it should have a `MuJoCo.app` file that you can drag into your /Application folder, so it becomes just like other apps in your Mac. You could then open up the app and drag xml files in it. Find more informationa in the [official documentation](https://mujoco.readthedocs.io/en/latest/programming/#getting-started).

### Install other packages
```
pip install -r requirements.txt
```

### Acquire OpenAI/Claude API Keys
This is required for prompting GPTs or Claude LLMs. You don't necessarily need both of them. Put your key string somewhere safely in your local repo, and provide a file path (something like `./roco/openai_key.json`) and load them in the scripts. Example code snippet:
```
import openai  
openai.api_key = YOUR_OPENAI_KEY

import anthropic
client = anthropic.Client(api_key=YOUR_CLAUDE_KEY)
streamed = client.completion_stream(...)  
```

## Usage 
### Run multi-robot dialog on the PackGrocery Task using the latest GPT-4 model
```
$ conda activate roco
(roco) $ python run_dialog.py --task pack -llm gpt-4
```


## Contact
Please direct to [Mandi Zhao](https://mandizhao.github.io). 
If you are interested in contributing or collaborating, please feel free to reach out! I'm more than happy to chat and brainstorm together. 

## Cite
```
@misc{mandi2023roco,
      title={RoCo: Dialectic Multi-Robot Collaboration with Large Language Models}, 
      author={Zhao Mandi and Shreeya Jain and Shuran Song},
      year={2023},
      eprint={2307.04738},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```