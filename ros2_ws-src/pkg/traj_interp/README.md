# Trajectory Interpolator Package

ROS2 package providing two different trajectory interpolation approaches for autonomous drone flight with integrated teleop support.

## Overview

The `traj_interp` package offers **two distinct interpolator nodes** for different navigation requirements:

### 1. Standard Trajectory Interpolator (`trajectory_interpolator`)
- **Classic Path Following**: Direct interpolation of planned paths
- **Filter-based Smoothing**: Second-order filter with jerk/acceleration limiting
- **Simple Navigation**: Best for obstacle-free environments
- **High Performance**: Lower computational overhead

### 2. Local Trajectory Interpolator (`local_trajectory_interpolator`)
- **Local Planning**: Integrated Artificial Potential Fields (APF) for real-time obstacle avoidance
- **Dynamic Replanning**: Reactive navigation with collision avoidance
- **Complex Environments**: Designed for cluttered spaces with dynamic obstacles
- **Safety Features**: Multiple safety layers with repulsive forces

Both interpolators share:
- **Teleop Integration**: Seamless switching between autonomous and manual control
- **Safety Systems**: Automatic arming/disarming and position synchronization
- **Transform Support**: Coordinate frame transformations (map ↔ odom)

## Node Selection Guide

### Use **Standard Trajectory Interpolator** when:
- ✅ **Clear environments** with minimal obstacles
- ✅ **High-quality global paths** from path planner
- ✅ **Performance priority** (lower CPU usage)
- ✅ **Predictable environments** with static obstacles

### Use **Local Trajectory Interpolator** when:
- ✅ **Cluttered environments** with many obstacles
- ✅ **Dynamic obstacles** that move during flight
- ✅ **Safety priority** over performance
- ✅ **Local sensors** available (depth cameras, lidar)
- ✅ **Real-time reactivity** needed

## Standard Trajectory Interpolator Features

### 🎯 Path Following
- **Path Resampling**: Automatic waypoint density optimization
- **Smooth Interpolation**: Second-order filter with jerk/acceleration limiting
- **Multi-Waypoint Support**: Sequential waypoint following with early advancement
- **Position Hold**: Maintains current position when no trajectory active

## Local Trajectory Interpolator Features

### 🧠 Local Planning
- **Artificial Potential Fields**: Real-time force-based navigation
- **Obstacle Avoidance**: Repulsive forces from multiple sensor sources
- **Dynamic Replanning**: Continuous path adjustment during flight
- **Multi-Sensor Fusion**: Combines map-based and body-frame obstacle data

### 🛡️ Enhanced Safety
- **Collision Avoidance**: Real-time obstacle detection and avoidance
- **Force Balancing**: Smooth balance between attractive and repulsive forces
- **Configurable Safety Zones**: Adjustable influence ranges and force parameters
- **Emergency Stopping**: Automatic stop on collision detection

## Common Features (Both Interpolators)

### 🎮 Teleop Integration
- **Flag-Based Coordination**: Responds to teleop_active flag from move manager
- **Velocity Integration**: Converts incremental velocity commands to position setpoints
- **Position Synchronization**: Prevents jumps during mode transitions
- **Single Publication**: Eliminates dual publication issues

### 🛡️ Safety Features
- **Automatic Arming**: Arms vehicle when trajectory starts and drone is landed
- **Landing Detection**: Disarms automatically when landing is detected during flight
## Topics

### Common Topics (Both Interpolators)

**Subscribed**:
- `/trajectory_path` (nav_msgs/Path): Trajectory waypoints to follow
- `/px4/odometry/out` (nav_msgs/Odometry): Vehicle position and attitude feedback
- `/move_manager/teleop_active` (std_msgs/Bool): Teleop mode activation flag
- `/teleop/velocity_increments` (geometry_msgs/Twist): Velocity commands from teleop
- `/fmu/out/vehicle_control_mode` (px4_msgs/VehicleControlMode): Vehicle state monitoring
- `/fmu/out/vehicle_land_detected` (px4_msgs/VehicleLandDetected): Landing detection

**Published**:
- `/px4/trajectory_setpoint_enu` (trajectory_msgs/MultiDOFJointTrajectoryPoint): Smooth trajectory setpoints
- `/trajectory_interpolator/status` (std_msgs/String): Current interpolator state
- `/fmu/in/vehicle_command` (px4_msgs/VehicleCommand): Arm/disarm and mode commands

### Additional Topics (Local Trajectory Interpolator Only)

**Subscribed**:
- `/local_grid_obstacle` (sensor_msgs/PointCloud2): Map-based obstacle cloud
- `/depth_camera/points` (sensor_msgs/PointCloud2): Body-frame obstacle cloud
- `/local_planner_enable` (std_msgs/Bool): Enable/disable local planning
- `/collision_detected` (std_msgs/Bool): External collision detection signals

**Published**:
- `/local_traj_interp/status` (std_msgs/String): Local planner status
- `/visualization_marker_array` (visualization_msgs/MarkerArray): Debug visualization

## Operation Modes

### Autonomous Mode
1. Receives trajectory path on `/trajectory_path`
2. Resamples waypoints for optimal spacing (40cm default)
3. Transforms waypoints from map to odom frame
4. Generates smooth interpolated setpoints using second-order filter
5. Publishes trajectory setpoints to PX4
6. Advances to next waypoint at half-distance for continuous flow

### Teleop Mode
1. Activated by teleop_active flag from move manager
2. Receives velocity increments from enhanced_teleop node
3. Integrates velocity commands to update target position
4. Maintains current position when velocity commands are zero
5. Synchronizes reference and command positions for smooth publication
6. Returns to position hold when teleop deactivated

## Usage

### Selecting Interpolator

#### Standard Trajectory Interpolator
```bash
# Launch standard interpolator
ros2 run traj_interp trajectory_interpolator

# With configuration file
ros2 run traj_interp trajectory_interpolator --ros-args --params-file config/trajectory_interpolator.yaml
```

#### Local Trajectory Interpolator
```bash
# Launch local planner interpolator
ros2 run traj_interp local_trajectory_interpolator

# With configuration file
ros2 run traj_interp local_trajectory_interpolator --ros-args --params-file config/local_trajectory_interpolator.yaml
```

### Basic Operation
```bash
# Send trajectory path (both interpolators)
ros2 topic pub /trajectory_path nav_msgs/Path "..."

# Monitor standard interpolator status
ros2 topic echo /trajectory_interpolator/status

# Monitor local interpolator status
ros2 topic echo /local_traj_interp/status

# Enable/disable local planning (local interpolator only)
ros2 topic pub /local_planner_enable std_msgs/Bool "data: true"
```

### Integration with Move Manager
Both interpolators automatically integrate with the move manager system:
- Respond to teleop_active flags
- Process paths generated by move manager
- Coordinate with enhanced_teleop for manual control

The choice of interpolator can be configured in the move manager launch files.

## Configuration Examples

### Environment-Based Selection

#### Clear Environment (Standard Interpolator)
```yaml
trajectory_interpolator:
  ros__parameters:
    control_frequency: 50.0
    waypoint_tolerance: 0.15
    ref_vel_max: 1.5          # Higher speed allowed
    ref_acc_max: 1.2
    parent_transform: "map"
    child_transform: "odom"
    do_transform: true
```

#### Cluttered Environment (Local Interpolator)
```yaml
local_trajectory_interpolator:
  ros__parameters:
    control_frequency: 50.0
    max_lin_speed: 0.8        # Conservative speed
    safety_distance: 1.5      # Larger safety margin
    k_attractive: 1.0
    k_repulsive: 0.9          # Balanced forces
    obstacle_influence_range: 1.8
    max_repulsive_force: 1.5  # Prevent force dominance
    
    # Obstacle sources
    obstacle_cloud_map_topic: "/local_grid_obstacle"
    obstacle_cloud_body_topic: "/depth_camera/points"
```

## Troubleshooting

### Interpolator Selection Issues
1. **Standard interpolator crashes into obstacles**: Switch to local interpolator
2. **Local interpolator too slow/conservative**: Switch to standard interpolator
3. **Oscillating behavior**: Reduce `k_repulsive` in local interpolator
4. **Not following path**: Increase `k_attractive` in local interpolator

### Common Issues (Both Interpolators)
1. **No setpoints published**: Check if odometry is being received
2. **Transform errors**: Verify TF tree and frame names
3. **Teleop not smooth**: Check velocity increment timing and synchronization

### Standard Interpolator Specific
1. **Jerky motion**: Adjust filter parameters (omega, zeta)
2. **Too slow**: Increase `ref_vel_max` and `ref_acc_max`

### Local Interpolator Specific
1. **Drone stuck near obstacles**: Reduce `safety_distance` or `obstacle_influence_range`
2. **Forces too strong**: Reduce `max_repulsive_force`
3. **Not avoiding obstacles**: Check obstacle topics and increase `k_repulsive`

### Debug Commands
```bash
# Check standard interpolator status
ros2 topic echo /trajectory_interpolator/status

# Check local interpolator status
ros2 topic echo /local_traj_interp/status

# Monitor setpoints (both)
ros2 topic echo /px4/trajectory_setpoint_enu

# Debug local planner forces
ros2 topic echo /visualization_marker_array

# Check obstacle clouds (local interpolator)
ros2 topic echo /local_grid_obstacle
ros2 topic echo /depth_camera/points

# Check transform tree
ros2 run tf2_tools view_frames
```

## License

This trajectory interpolation system is part of the Drone Manager package.

### Authors
- **Simone D'Angelo** - simone.dangelo@unina.it
- **Francesca Pagano** - francesca.pagano@unina.it  
- **Vincenzo Scognamiglio** - vincenzo.scognamiglio2@unina.it

**PRISMA LAB** - University of Naples Federico II

---

**Note**: This system is designed for research and development purposes. Always follow local aviation regulations and safety guidelines when operating autonomous vehicles.