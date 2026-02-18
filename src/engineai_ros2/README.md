# engineai_ros2

ROS 2 integration utilities for the EngineAI robot family (PM‑V2 today, designed to be extended to other platforms such as T800).

This package is developed and typically built inside the upstream workspace repository:

- https://github.com/engineai-robotics/engineai_ros2_workspace

---

## What is included

### 1) `engineai_hardware_interface` (ros2_control system plugin)

- Shared library: `libengineai_hardware_interface.so`
- Plugin XML: `engineai_hardware_interface.xml`
- Current plugin class (PM‑V2):
  - `pm_v2_hardware_interface::PM_V2_SystemPositionOnlyHardware`

This naming keeps the **plugin library generic** (`engineai_hardware_interface`) so future robots (e.g., T800) can add additional plugin classes without renaming the package-level interface.

### 2) `joint_command_mux` (node)

Merges two `interface_protocol/msg/JointCommand` topics:

- RL command topic (base + publish clock)
- ROS command topic (optional upper‑body override)

Publishing rate follows the **RL topic** (the RL callback is the publish clock).  
Supports smooth transitions when switching the upper‑body source.

### 3) `engineai_cmd_vel_controller` (node)

Converts `geometry_msgs/msg/Twist` (`cmd_vel`) into `interface_protocol/msg/GamepadKeys`
(EngineAI analog command message used by the downstream interface stack).  
Publishes a STOP message on shutdown signals.

---

## Workspace setup (recommended)

```bash
mkdir -p ~/engineai_ws
cd ~/engineai_ws
git clone https://github.com/engineai-robotics/engineai_ros2_workspace.git
cd engineai_ros2_workspace

# Install build dependencies

# Refer to the workspace doc for details:
# https://github.com/engineai-robotics/engineai_ros2_workspace/tree/community?tab=readme-ov-file#software-dependencies
# https://github.com/engineai-robotics/engineai_ros2_workspace/tree/community?tab=readme-ov-file#build-instructions


# If the workspace provides a .repos file, import/update dependencies like this:
# vcs import src < engineai_ros2_workspace.repos

# Install dependencies (adjust ROS_DISTRO if needed)
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --packages-up-to engineai_ros2

# Source the setup after building
source install/setup.bash
```

---
## 🚀 Runtime Usage (Full Pipeline)

The EngineAI control stack requires **three components**:

1. MuJoCo simulator  
2. RL control example  
3. ROS2 control + joint command mux  

These must be started in the correct order.

---

### 1.  Start MuJoCo Simulator

```bash
cd ~/engineai_ws/engineai_ros2_workspace
source install/setup.bash

ros2 launch mujoco_simulator mujoco_simulator.launch.py
```

Verify that the base hardware topics exist:

```bash
ros2 topic list | grep joint
```

You should see something like:

```
/hardware/joint_command
/hardware/joint_state
```

If these topics are missing, the simulator is not running correctly.

### 2. Start RL Example
```bash
ros2 launch interface_example rl_basic_example.launch.py joint_command_topic:=/hardware/joint_command_rl
```

Verify RL command output:

```
ros2 topic echo -n 1 /hardware/joint_command_rl
```
If nothing appears, RL example is not running, Namespace mismatch, or QoS mismatch

### 3. Start engineai_ros2 (ROS2 Control + Mixer)

```bash
ros2 launch engineai_ros2 ros2_control_pm_v2.launch.py
```
After launch, you should see:

```
/hardware/joint_command_ros
/hardware/joint_command_rl
/hardware/joint_command
```
The joint_command_mux node merges ROS and RL commands and publishes to /hardware/joint_command


### Switching Upper Body Source
The mixer supports smooth runtime switching between control sources.

Use RL for all joints:

```
ros2 service call /joint_command_mixer/set_upper_rl std_srvs/srv/SetBool "{data: true}"
```
Use ROS for upper body (RL for lower body):
```
ros2 service call /joint_command_mixer/set_upper_rl std_srvs/srv/SetBool "{data: false}"
```

Transitions are smoothed automatically using linear blending.

### Optional: Launch All Components Automatically
You may create a helper script:

```bash
#!/bin/bash
source ~/engineai_ws/engineai_ros2_workspace/install/setup.bash

gnome-terminal -- ros2 launch mujoco_simulator mujoco_simulator.launch.py
sleep 3
gnome-terminal -- ros2 launch interface_example rl_basic_example.launch.py joint_command_topic:=/hardware/joint_command_rl
sleep 3
gnome-terminal -- ros2 launch engineai_ros2 ros2_control_pm_v2.launch.py
```
Save as:

```bash
start_engineai_pipeline.sh
```
Make executable:

```bash
chmod +x start_engineai_pipeline.sh
```

Run:
```bash
./start_engineai_pipeline.sh
```

---

## joint_command_mux

### Topics

- Subscribes:
  - `topic_rl` (default: `/hardware/joint_command_rl`)  
  - `topic_ros` (default: `/hardware/joint_command_ros`)
- Publishes:
  - `topic_out` (default: `/hardware/joint_command`)

### Parameters

- `upper_source` (`string`, default: `ros`)
  - `ros`: override upper‑body joints from ROS topic
  - `rl`: do not override (RL provides all joints)
- `transition_time` (`double`, default: `0.3`)
  - Seconds to blend when switching `upper_source`.
- `blend_position_only` (`bool`, default: `true`)
  - If `true`, only `position[]` is blended/overridden for safety.
- `joint_names` (`string[]`, default: `[]`)
  - Joint list in the **same order** as `JointCommand` arrays.

### Service

- `~/set_upper_rl` (`std_srvs/srv/SetBool`)
  - `data=true`  → `upper_source=rl`
  - `data=false` → `upper_source=ros`

---

## engineai_cmd_vel_controller

### Topics

- Subscribes:
  - `cmd_vel` (default: `/cmd_vel`)
- Publishes:
  - `interface_protocol/msg/GamepadKeys` (default: `/hardware/gamepad_keys`)

### Parameters

- `cmd_vel_topic` (`string`, default: `/cmd_vel`)
- `out_topic` (`string`, default: `/hardware/gamepad_keys`)
- `publish_rate_hz` (`double`, default: `50.0`)  
  (If set, a timer republishes the last command at this rate.)

---

## Per-joint stiffness/damping configuration (ros2_control)

For PM‑V2 hardware plugin parameters, use the `<ros2_control><hardware><param .../></hardware></ros2_control>` block.

Example (defaults + per-joint overrides):

```xml
<hardware>
  <plugin>pm_v2_hardware_interface/PM_V2_SystemPositionOnlyHardware</plugin>

  <param name="default_stiffness">20.0</param>
  <param name="default_damping">1.0</param>

  <!-- name=value CSV -->
  <param name="stiffness_by_joint">SHOU_P_L=10.0,ELBO_P_L=8.0</param>
  <param name="damping_by_joint">SHOU_P_L=0.8,ELBO_P_L=0.7</param>
</hardware>
```

---

## URDF / mesh path fix (RViz-friendly)

The original URDF used by the simulator stack contains mesh references like:

- `package://resource/...`

Those paths are not RViz-friendly in this package layout.

During CMake configure/build, we **copy the URDF + meshes from `mujoco_simulator`** into:

- `${PROJECT_NAME}/share/${PROJECT_NAME}/resource/...`

and rewrite the URDF references:

- `package://resource/...` → `package://engineai_ros2/resource/...`

This CMake-time rewrite is intentionally kept because it makes RViz display possible using the installed URDF.

---


## License

Apache-2.0
