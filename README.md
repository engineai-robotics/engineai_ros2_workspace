# EngineAI ROS: Development Guide with ROS2 Interface for EngineAI Robot

[中文](README_CN.md) | English

<img src="docs/images/pm01.jpg" alt="EngineAI Logo" width="600">

## Overview

EngineAI ROS is a ROS2 Workspace that includes the ROS2 interface and example code for EngineAI robots. This repository provides ROS use cases. To learn about the SDK, see [EngineAI Robotics SDK](https://github.com/engineai-robotics/engineai_robotics_sdk).

## ROS2 Interface Protocol

[Interface Protocol Description](src/interface_protocol/README.md)

## Architecture

### Workspace Structure

```bash
└── src
    ├── interface_example # Interface examples, recommended to run on Nezha
    ├── interface_protocol # Interface protocols, common modules
    ├── third_party # Third-party libraries
    ├── simulation # Simulation environment running on host
```

### Robot Architecture & Operation (Required Reading)

See [EngineAI Open Source Knowledge Base](https://dx3a2bminsq.feishu.cn/wiki/P6izwRvaDi9Zo3kR0okc2kg7n2c)

## Build & Run

### Development Environment Requirements for Your Host PC

#### Basic Environment

- Ubuntu 22.04
- ROS2 Humble Desktop
- GCC >= 11
- CMake >= 3.22
- Python >= 3.10

#### Software Dependencies

```
sudo apt update
sudo apt install rsync sshpass openssh-client libglfw3-dev libxinerama-dev libxcursor-dev
sudo apt install ros-dev-tools ros-humble-rmw-cyclonedds-cpp ros-humble-ros-base
```

#### Environment Variables

Environment variable configuration:

```bash
# Set ROS_DOMAIN_ID to 69 (EngineAI robot domain ID)
export ROS_DOMAIN_ID=69
# Set ROS_LOCALHOST_ONLY to 0 to allow ROS2 nodes to communicate on the local network
export ROS_LOCALHOST_ONLY=0
# Set RMW_IMPLEMENTATION to rmw_cyclonedds_cpp (Cyclone DDS as ROS2 middleware)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

You can also add these to `~/.bashrc` with:

```bash
echo -e '\nexport ROS_DOMAIN_ID=69\nexport ROS_LOCALHOST_ONLY=0\nexport RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc && source ~/.bashrc
```

### Connect to the Robot

### Simulation Repository

An independent simulation environment is now available. We recommend using [EngineAI Robot Simulation Environment](https://github.com/engineai-robotics/engineai_robot_simulation_runtime) for simulation testing.

#### Simulator (Currently only supports simulation in joint bridge mode)

```bash
# in host
# terminal 1
./src/third_party/install.sh
./scripts/build_nodes.sh sim
source install/setup.bash
ros2 launch mujoco_simulator mujoco_simulator.launch.py
```

> **IMPORTANT**: When running simulation, either do not connect to the physical robot or set `ROS_LOCALHOST_ONLY=1` in your environment to prevent accidental connections.

#### Real Robot

Connect to the robot’s internal network via the Ethernet port on the back. The internal network is 192.168.0.0/24. You can check connectivity with:

```bash
# in host
ping 192.168.0.163
```

> **WARNING**: Do not use USB network adapters; they may cause ROS2 communication issues. Use the built-in Ethernet port for reliable communication with the robot.

### Run Example Code

#### Body Velocity Control Example

1. Enter any robot walking mode (e.g. Basic Walk).
2. Run the example:

```
# in host
./scripts/build_nodes.sh example
source install/setup.bash
python src/interface_example/scripts/body_velocity_control_example.py
```

#### RL Walking Policy Deployment Example

##### Introduction

EngineAI provides an open-source RL training framework [EngineAI Gym](https://github.com/engineai-robotics/engineai_gym). You can train your own RL controller with it and follow the steps below for sim2sim and on-robot deployment.

By default, a walking policy is loaded from `src/interface_example/config/pm01/rl_basic/basic/policies/pm01_v2_rough_ppo_42obs.mnn`. You can run this example in the simulator or on the real robot. You can replace it with your own policy trained with the open-source framework.

##### Deploy: Run joint_test_example on your host

1. Enter passive mode.
2. Enter joint bridge mode
  - Joint bridge mode allows control via joint commands.
  - With the robot in passive mode, press the Joint Bridge button on the remote control.
  - Verify the robot status:
  - Confirm that the motion state is `joint_bridge` before proceeding.
3. Run the joint test example:
  ```bash
   # in host
   python3 src/interface_example/scripts/joint_test_example.py
  ```

##### Deploy: Run rl_basic_example on your host

1. Enter pd-stand mode
  - Ensure the robot is on flat ground with enough space around it.
  - Use the remote to enter pd-stand mode.
  - Make sure the robot is stable and standing before proceeding.
2. Enter joint bridge mode
  - With the robot in pd-stand mode, press the Joint Bridge button on the remote.
  - Verify the robot status:
  - Confirm that the motion state is `joint_bridge` before proceeding.
3. Run the RL example
  - **SAFETY**: Keep everyone at a safe distance. Be ready to stop the robot quickly (emergency stop or passive mode) if it behaves unexpectedly.
  - Launch the example:

> **NOTE**: If the robot falls in simulation, you may need to reset the simulator using the reset button.



### Compile the Code on the Target Board

If you do not have a PC development environment, you can build and run on the robot’s onboard computer.

#### Sync the Code to the Board

```bash
# in host
./scripts/sync_src.sh orin
```

#### Build the Workspace

SSH into the Orin board and go to the workspace directory:

```bash
# in orin
cd ~/source/engineai_workspace
./scripts/build_nodes.sh
```

Usage on the board is the same as on the host; refer to the host instructions above.