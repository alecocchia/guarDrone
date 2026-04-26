# Enhanced Teleop Node

ROS2 node providing coordinated manual control for drone operations with flag-based activation and velocity increment publishing.

## Overview

The Enhanced Teleop Node provides:
- **Coordinated Control**: Responds to teleop_active flags from move manager
- **Velocity Increments**: Publishes incremental velocity commands for smooth control
- **Joystick Integration**: Standard joystick input processing with configurable mappings
- **Safety Features**: Automatic activation/deactivation based on system coordination

## Features

### 🎮 Control Integration
- **Flag-Based Activation**: Responds to `/move_manager/teleop_active` instead of manual start/stop
- **Velocity Increments**: Publishes on `/teleop/velocity_increments` for position integration
- **Axis Mapping**: Consistent with offboard_control mapping (axis 0=yaw, 1=z, 2=y, 3=x)
- **Configurable Scaling**: Adjustable velocity and yaw rate scaling

### 🛡️ Safety Features
- **Automatic Activation**: Starts when teleop_active flag is true
- **Clean Deactivation**: Stops when teleop_active flag is false
- **Deadzone Handling**: Configurable joystick deadzone for precision
- **Smooth Control**: Proportional control with configurable scaling

## Architecture

```
Joy Input → Enhanced Teleop → Velocity Increments → Trajectory Interpolator
     ↑              ↑                    ↓
Move Manager → Teleop Active Flag → Position Integration
```

## Topics

### Subscribed
- `/joy` (sensor_msgs/Joy): Joystick input data
- `/move_manager/teleop_active` (std_msgs/Bool): Activation flag from move manager

### Published
- `/teleop/velocity_increments` (geometry_msgs/Twist): Velocity commands for trajectory interpolator

## Parameters

### Control Parameters
```yaml
enhanced_teleop:
  max_velocity: 2.0              # Maximum velocity [m/s]
  max_angular_velocity: 1.0      # Maximum angular velocity [rad/s]
  deadzone: 0.1                  # Joystick deadzone threshold
  velocity_scale: 0.5            # Velocity scaling factor
  yaw_scale: 0.3                 # Yaw rate scaling factor
```

### Axis Mapping (consistent with offboard_control)
```yaml
  axis_x: 3                      # X velocity (right stick horizontal)
  axis_y: 2                      # Y velocity (right stick vertical)  
  axis_z: 1                      # Z velocity (left stick vertical)
  axis_yaw: 0                    # Yaw velocity (left stick horizontal)
```

## Usage

### Basic Operation
```bash
# Launch enhanced teleop node
ros2 run teleop_node enhanced_teleop_node

# Or use launch file
ros2 launch teleop_node enhanced_teleop.launch.py
```

### Integration with System
The node automatically integrates with the move manager system:
1. Listens for teleop_active flag from move manager
2. Activates when flag is true and joystick is available
3. Publishes velocity increments for trajectory interpolator
4. Deactivates when flag is false

### Configuration
```bash
# Launch with custom parameters
ros2 launch teleop_node enhanced_teleop.launch.py \
  velocity_scale:=0.8 \
  yaw_scale:=0.4 \
  deadzone:=0.15
```

## Control Mapping

### Standard Xbox/PS4 Controller
- **Left Stick Horizontal** (axis 0): Yaw rotation
- **Left Stick Vertical** (axis 1): Z movement (up/down)
- **Right Stick Horizontal** (axis 3): X movement (forward/backward)
- **Right Stick Vertical** (axis 2): Y movement (left/right)

## Troubleshooting

### Common Issues
1. **Teleop not activating**: Check if joy node is running and teleop_active flag is true
2. **No joystick response**: Verify joystick connection and axis mapping
3. **Jerky movement**: Adjust velocity_scale and deadzone parameters
4. **Wrong axis mapping**: Check controller type and update axis parameters

### Debug Commands
```bash
# Check teleop activation
ros2 topic echo /move_manager/teleop_active

# Monitor joystick input
ros2 topic echo /joy

# Check velocity increments
ros2 topic echo /teleop/velocity_increments

# Test joystick axes
ros2 run rqt_plot rqt_plot /joy/axes[0] /joy/axes[1] /joy/axes[2] /joy/axes[3]
```

### Performance Notes
- **Low Latency**: Direct joystick input processing without buffering
- **Smooth Integration**: Velocity increments integrated by trajectory interpolator
- **Resource Efficient**: Minimal CPU usage when inactive
- **Robust Operation**: Handles joystick disconnect/reconnect gracefully

## Dependencies
- ROS2 Humble
- sensor_msgs (Joy messages)
- geometry_msgs (Twist messages) 
- std_msgs (Bool messages)
- px4_msgs (PX4 integration)

## License
MIT License
```yaml
# In sim_params.yaml or params.yaml
max_vel: 2.0                # Maximum velocity (teleop uses 50%)
max_acc: 1.0                # Maximum acceleration
use_key_input: 1.0          # Enable keyboard interface
```

### Joystick Configuration
```bash
# Verify joystick device
ls /dev/input/js*

# Test joystick functionality
jstest /dev/input/js0

# Check joy_node output
ros2 topic echo /joy
```

## Troubleshooting

### Common Issues

#### 1. Teleop Won't Activate
```bash
# Check joy_node status
ros2 topic list | grep joy
ros2 topic hz /joy

# Verify _joy_available flag is set
# Look for "Joy node detected" in offboard_control logs
```

#### 2. Erratic Movement
- Check joystick calibration with `jstest`
- Verify stick dead zones are appropriate
- Ensure stable joystick connection

#### 3. Mode Switching Issues
```bash
# Check current mode status
ros2 topic echo /leo/drone/plan_status

# Verify position continuity
ros2 topic echo /fmu/in/trajectory_setpoint
```

### Debug Information
```bash
# Monitor teleop activation
# Look for these log messages in offboard_control:
# - "Joy node detected, enabling teleop capability"
# - "Entering teleop mode"
# - "Exiting teleop mode due to new command"

# Check joystick data
ros2 topic echo /joy --no-arr

# Monitor trajectory setpoints
ros2 topic echo /fmu/in/trajectory_setpoint
```

## Advanced Features

### Smooth Transitions
The system ensures continuity when switching modes:
```cpp
// On teleop exit, position is preserved
_prev_sp = _teleop_position;
_prev_yaw_sp = _teleop_yaw;
_prev_att_sp = matrix::Quaternionf(matrix::Eulerf(0, 0, _teleop_yaw));
```

### Thread Safety
- Atomic flag operations for mode switching
- Thread-safe position updates
- Synchronized trajectory publishing

### Integration with Autonomous Modes
- Seamless entry from any autonomous mode
- Preserved trajectory state on exit
- Intelligent fallback mechanisms

## Safety Guidelines

⚠️ **Important Safety Notes**:
- Always test in simulation first
- Maintain visual contact with drone during teleop
- Keep emergency stop readily available
- Verify joystick connection stability before flight
- Ensure adequate flight space for manual control

## License

This teleop control system is part of the Trajectory Planner package.

### Authors
- **Simone D'Angelo** - simone.dangelo@unina.it
- **Francesca Pagano** - francesca.pagano@unina.it  
- **Vincenzo Scognamiglio** - vincenzo.scognamiglio2@unina.it

**PRISMA LAB** - University of Naples Federico II

---

**Note**: This system is designed for research and development purposes. Always follow local aviation regulations and safety guidelines when operating autonomous vehicles.
