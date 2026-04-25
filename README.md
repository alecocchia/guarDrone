# UAV MOTION STACK

This repository contains tools and configurations for PX4 SITL (Software In The Loop) simulation and hardware-specific deployment.

## Architecture Overview

The system consists of several modular ROS2 packages, each with a specific responsibility:

```
uav_motion_stack/
├── ros2_ws-src/                    # ROS2 workspace with modular packages
│   ├── aruco_detector_ocv_ros2/    # opencv-based aruco detector (submodule)
│   ├── drone_odometry2/            # Vehicle odometry publisher (submodule)
│   ├── path_planner/               # 3D trajectory planning (submodule)
│   ├── teleop_node/                # Teleoperation control (submodule)
│   ├── babyk_drone_manager/        # Drone state management and safety (submodule)
│   └── traj_interp/                # Trajectory interpolation with PX4 (submodule)
├── docker/               # Docker configurations
├── models/               # Custom Gazebo models
├── worlds/               # Gazebo worlds for simulation
├── PX4-Autopilot/        # PX4 firmware 
└── PX4_neabotics/        # PX4 custom firmware 
```

## System Requirements

- **Docker**: For isolated development environment
- **ROS2 Humble**: Robotics framework
- **PX4 v1.14+**: Autopilot firmware
- **Gazebo Garden**: 3D simulator
- **Eigen3**: Mathematical library for matrix operations

## Installation and Setup

A step by step series of examples that tell you how to get a development environment running:

### 1. Repository Clone
```bash
git clone --recursive https://github.com/Prisma-Drone-Team/uav_motion_stack.git -b paper_stable
cd uav_motion_stack
```

### 2. Clone PX4 Firmware (Optional)
> **Note:** This step is completely optional given the current state of the repository. The custom firmware in step 3 is sufficient for the full stack.

```bash
git clone --single-branch -b release/1.14 git@github.com:PX4/PX4-Autopilot.git --recursive
```

### 3. Clone PX4 Neabotics (Required for Plug-and-Play)
> **Important:** This custom firmware is **required** for the plug-and-play UAV motion stack functionality.

```bash
git clone --single-branch -b feature/diffgains_fix_servo_k https://github.com/Prisma-Drone-Team/Px4_hcore_autopilot.git PX4_neabotics --recursive
```

### 4. Build Docker Image
```bash
cd docker
docker build -t leo-img -f px4_humble_dockerfile.txt .
```
> **Note:** The container uses Gazebo Garden simulator. A Dockerfile for Gazebo Classic is also available but its integration into the stack is deprecated.

### 5. Run Container
```bash
./run_cnt.sh
```

## Development Configuration

### ROS2 Workspace Build
```bash
cd ros2_ws
source install/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### Main Dependencies
```xml
<!-- Common package.xml -->
<depend>rclcpp</depend>
<depend>px4_msgs</depend>
<depend>nav_msgs</depend>
<depend>geometry_msgs</depend>
<depend>trajectory_msgs</depend>
<depend>tf2</depend>
<depend>tf2_ros</depend>
<depend>eigen3_cmake_module</depend>
```

## Usage in simulation with TMUX
```bash
cd ros2_ws
tmuxp load src/pkg/babyk_drone_manager/utils/simulation.yml
```
### Terminate the simulation
**Note:** kill the PX4 firmware and in the same terminal type: 
```bash
tmux kill-server
```
## Package Documentation

Each ROS2 package used in this system is documented in its own specific README:

- **traj_interp**: Detailed documentation of the interpolation algorithm and PX4 integration
- **drone_odometry2**: Odometry message conversion specifications
- **path_planner**: 3D planning and obstacle avoidance algorithms
- **teleop_node**: Manual control configuration and interfaces
- **babyk_drone_manager**: Safety system and state monitoring

Refer to the README.md file in each package folder for technical details.

## Important Notes

**PX4 Firmware**: The PX4-Autopilot and PX4_neabotics firmwares must be downloaded separately and are used exclusively for SITL simulation. They are not required for deployment on real hardware.

**PX4_neabotics**: This firmware is specialized for tiltrotor drones and optimized for the Leonardo Drone Contest field, with specific improvements for tiltrotor flight dynamics.

## ROS2 Packages

### 🛸 traj_interp
**Trajectory interpolator with complete PX4 integration**

Implements the algorithm for smooth trajectory interpolation with integrated PX4 offboard control.

**Key Features:**
- Smooth trajectory interpolation with jerk/acceleration limiting
- Complete PX4 integration: arming/disarming, offboard mode
- Automatic heading calculation based on movement direction
- Smart arming: only on first path or after landing
- Auto-disarming on land detection
- Automatic PX4 mode management

#### Base Trajectory Interpolator Algorithm

The core algorithm implements a smooth trajectory interpolator with velocity, acceleration, and jerk limiting for each axis. Here's the step-by-step process:

**Input Parameters:**
- UAV position **p**, desired goal position **p^cmd**
- Parameters: (ωᵢ, ζᵢ, aᵢᵐᵃˣ, vᵢᵐᵃˣ, jᵢᵐᵃˣ) for i ∈ {x,y,z}
- Timestep Δt
- Initial conditions: pᵢʳᵉᶠ(0) = pᵢᶜᵐᵈ, vᵢʳᵉᶠ(0) = 0, aᵢʳᵉᶠ(0) = 0

**Algorithm Steps:**

For each axis i ∈ {x, y, z}:

1. **Calculate desired acceleration:**
   ```
   aᵢᵈᵉˢ(t) = ωᵢ² × (pᵢᶠᵇ(t) - pᵢʳᵉᶠ(t)) - 2ζᵢωᵢvᵢʳᵉᶠ(t)
   ```

2. **Compute jerk:**
   ```
   jᵢ = (aᵢᵈᵉˢ - aᵢʳᵉᶠ) / Δt
   ```

3. **Apply jerk limiting:**
   ```
   if |jᵢ| > jᵢᵐᵃˣ:
       jᵢ = sign(jᵢ) × jᵢᵐᵃˣ
   
   aᵢᵈᵉˢ = aᵢʳᵉᶠ + jᵢ × Δt
   ```

4. **Apply acceleration limiting:**
   ```
   if |aᵢᵈᵉˢ| > aᵢᵐᵃˣ:
       aᵢʳᵉᶠ = sign(aᵢᵈᵉˢ) × aᵢᵐᵃˣ
   else:
       aᵢʳᵉᶠ = aᵢᵈᵉˢ
   ```

5. **Integrate to compute velocity:**
   ```
   vᵢᵈᵉˢ = vᵢʳᵉᶠ + aᵢʳᵉᶠ × Δt
   ```

6. **Apply velocity limiting:**
   ```
   if |vᵢᵈᵉˢ| > vᵢᵐᵃˣ:
       vᵢʳᵉᶠ = sign(vᵢᵈᵉˢ) × vᵢᵐᵃˣ
   else:
       vᵢʳᵉᶠ = vᵢᵈᵉˢ
   ```

7. **Integrate to compute position:**
   ```
   pᵢʳᵉᶠ = pᵢʳᵉᶠ + vᵢʳᵉᶠ × Δt
   ```

This algorithm ensures smooth trajectory following by limiting jerk (rate of acceleration change), acceleration, and velocity independently for each axis, resulting in physically feasible and smooth drone movements.

**Main Topics:**
- **Subscriber:** `/path` (nav_msgs/Path) - Trajectory to follow
- **Publisher:** `/px4_trajectory` (trajectory_msgs/MultiDOFJointTrajectory) - Interpolated trajectory
- **Publisher:** `/fcu/in/vehicle_command` - PX4 commands (arm/disarm)
- **Publisher:** `/fcu/in/offboard_control_mode` - Offboard control mode
- **Subscriber:** `/fcu/out/vehicle_control_mode` - Vehicle mode status
- **Subscriber:** `/fcu/out/vehicle_land_detected` - Landing status

### 📡 drone_odometry2
**Vehicle odometry publisher**

Converts PX4 status messages to standard ROS2 odometry.

**Main Topics:**
- **Subscriber:** `/fcu/out/vehicle_odometry` (px4_msgs/VehicleOdometry)
- **Publisher:** `/odom` (nav_msgs/Odometry)

### 🗺️ path_planner  
**3D trajectory planner**

Generates optimized 3D paths for drones with obstacle avoidance.

**Main Topics:**
- **Subscriber:** `/goal_pose` (geometry_msgs/PoseStamped) - Target goal
- **Publisher:** `/path` (nav_msgs/Path) - Planned trajectory

### 🎮 teleop_node
**Teleoperation control**

Interface for manual drone control via keyboard/joystick.

**Main Topics:**
- **Subscriber:** `/cmd_vel` (geometry_msgs/Twist) - Velocity commands
- **Publisher:** `/goal_pose` (geometry_msgs/PoseStamped) - Target pose

### 🛡️ babyk_drone_manager
**State management and safety**

Monitors drone status and implements safety functions. Implements the communication layer with the GCS.

**Main Topics:**
- **Subscriber:** `/seed_pdt_drone/command` (std_msgs/String) - new task primitive received
- **Publisher:** `/seed_pdt_drone/status` (std_msgs/String) - task status to GCS

