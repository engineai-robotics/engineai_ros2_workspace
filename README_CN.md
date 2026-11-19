# EngineAI ROS：EngineAI 机器人 ROS2 接口开发指南

中文 | [English](README.md)

## 概述

EngineAI ROS 是一个 ROS2 Workspace，包含了 EngineAI 机器人的 ROS2 接口和一些示例代码。本仓库是 ROS 用例仓库。如需了解 SDK，请参阅 [EngineAI Robotics SDK](https://github.com/engineai-robotics/engineai_robotics_sdk)。

## ROS2 接口协议

[接口协议说明](src/interface_protocol/README.md)

## 架构

### 工作空间结构

```bash
└── src
    ├── interface_example # Interface examples, recommended to run on Nezha
    ├── interface_protocol # Interface protocols, common modules
    ├── third_party # Third-party libraries
    ├── simulation # Simulation environment running on host
```

### 机器人架构&操作说明（必读）

查阅[EngineAI开源知识库](https://dx3a2bminsq.feishu.cn/wiki/P6izwRvaDi9Zo3kR0okc2kg7n2c)

## 构建与运行

### 主机开发环境要求

#### 基础环境

- Ubuntu 22.04
- ROS2 Humble Desktop
- GCC >= 11
- CMake >= 3.22
- Python >= 3.10

#### 软件依赖

```
sudo apt update
sudo apt install rsync sshpass openssh-client libglfw3-dev libxinerama-dev libxcursor-dev
sudo apt install ros-dev-tools ros-humble-rmw-cyclonedds-cpp ros-humble-ros-base
```

#### 环境变量

环境变量配置

```bash
# 设置ROS_DOMAIN_ID为69，这是EngineAI机器人的域ID
export ROS_DOMAIN_ID=69
# 设置ROS_LOCALHOST_ONLY为0，允许ROS2节点在本地网络中通信
export ROS_LOCALHOST_ONLY=0
# 设置RMW_IMPLEMENTATION为rmw_cyclonedds_cpp，使用Cyclone DDS作为ROS2的通信中间件
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

也可以通过下面的脚本一键加入环境变量到 ~/.bashrc :

```bash
echo -e '\nexport ROS_DOMAIN_ID=69\nexport ROS_LOCALHOST_ONLY=0\nexport RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc && source ~/.bashrc
```

### 连接机器人

### 仿真仓库

现已推出独立的仿真环境，建议使用 [EngineAI 机器人仿真环境](https://github.com/engineai-robotics/engineai_robot_simulation_runtime) 进行仿真测试。

#### 仿真器（当前仿真只支持关节桥接模式下的接口）

```bash
./src/third_party/install.sh
./scripts/build_nodes.sh sim
source install/setup.bash
ros2 launch mujoco_simulator mujoco_simulator.launch.py
```

> **重要**：运行仿真时，请勿连接真实机器人，或在环境中设置 `ROS_LOCALHOST_ONLY=1`，以免误连真实设备。

#### 真实机器人

通过机器人背后Ethernet接口连接到机器人内部网络，机器内部网段为192.168.0.0/24，你可以通过如下命令检查网络连接：

> **警告**：请勿使用 USB 网卡，可能导致 ROS2 通信异常。请使用主板自带网口与机器人可靠通信。

### 运行示例代码

#### 机身速度控制示例

1. 进入任意机器人行走模式
2. 运行示例

```
# in host
./scripts/build_nodes.sh example
source install/setup.bash
python src/interface_example/scripts/body_velocity_control_example.py
```

#### 强化学习行走策略部署示例

##### 示例介绍
EngineAI 提供开源 RL 训练框架 [EngineAI Gym](https://github.com/engineai-robotics/engineai_gym)，用户可使用该框架训练自己的 RL 控制器。

用户可按下列步骤，可以进行sim2sim和实机部署。

该示例默认会从目录 `src/interface_example/config/pm01/rl_basic/basic/policies/pm01_v2_rough_ppo_42obs.mnn`  加载一个行走策略。用户可以在仿真器中运行该示例，也可以在实机上运行该示例。


##### 部署：在主机上运行 rl_basic_example

1. 进入 pd-stand 模式
  - 确保机器人平稳放置在平坦地面，周围有足够空间
  - 使用遥控器进入 pd-stand 模式
  - 确认机器人站稳且稳定后再进行下一步
2. 进入关节桥接模式
  - 关节桥接模式支持通过关节指令控制
  - 在机器人处于 pd-stand 模式时，进入关节桥接模式
  - 机器人将进入关节桥接模式，可通过关节指令控制
  - 使用以下命令确认机器人状态：
  ```bash
  # in host
  ros2 topic echo /motion/motion_state
  ```
  - 确认运动状态为 `joint_bridge` 后再继续
3. 运行 RL 示例
  - **安全提醒**：确保所有人员与机器人保持安全距离
  - 若机器人动作异常，请能随时快速停止（按急停键或切回被动模式）
  - 使用以下命令启动示例：
  ```bash
  # in host
  ros2 launch interface_example rl_basic_example.launch.py
  ```
> **说明**：若仿真中机器人倒地，可按下复位按钮重置仿真器。



### 在机器人内置计算单元上编译代码

#### 将代码同步到板子

```bash
# in host
./scripts/sync_src.sh orin
```

#### 构建工作空间

首先通过 ssh 登录 Orin 板子并进入工作空间目录：

```bash
# in Orin
cd ~/source/engineai_workspace
./scripts/build_nodes.sh
```
接下来的使用方式与主机上相同，可参考主机上的使用方式。

