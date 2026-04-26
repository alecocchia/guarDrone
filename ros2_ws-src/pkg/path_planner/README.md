# Path Planner

**3D path planning system for drones with obstacle avoidance.**

## Overview

The `path_planner` is a specialized package for 3D path planning using OMPL (Open Motion Planning Library) and FCL (Flexible Collision Library). It focuses exclusively on path planning, delegating system management to the `babyk_drone_manager`.

## Key Features

- ✅ **3D Planning**: OMPL algorithms for three-dimensional spaces
- ✅ **Obstacle Avoidance**: FCL integration with octomap
- ✅ **Real-time Collision Checking**: Continuous collision monitoring
- ✅ **Replan Logic**: Automatic replanning with attempt limits
- ✅ **Path Resampling**: 40cm resampling for velocity control
- ✅ **Visualization**: RViz markers for paths and collisions
- **ROS 2 Integration**: Publishes paths in `nav_msgs::msg::Path` format compatible with `traj_interp`

## Architecture

```
path_planner/
├── src/
│   ├── path_planner_node.cpp     # Main node
│   └── planner.cpp               # OMPL/FCL logic
├── include/path_planner/
│   ├── path_planner_node.h       # Node header
│   └── planner.h                 # Planner header
├── launch/
│   └── path_planner.launch.py    # Configurable launch
└── config/
    ├── path_planner_params.yaml       # Real flight parameters
    ├── path_planner_simulation.yaml   # Simulation parameters
    ├── path_planner_flight.yaml       # Optimized flight parameters
    └── path_planner_params_old.yaml   # Parameter backup
```

## Path Planner Node

**Executable**: `path_planner_node`

### Topics

**Input**:
- `/move_base_simple/goal` - Goals from `babyk_drone_manager`
- `/px4/odometry/out` - Current drone position
- `/octomap_binary` - Obstacle map from RTABMap

**Output**:
- `/trajectory_path` - Planned path
- `/path_planner/status` - Planning status
- `/leo/drone/check_path` - Collision visualization markers

### States

- `IDLE` - Ready for new goals
- `PLANNING` - Planning in progress
- `PATH_PUBLISHED` - Path successfully published
- `PLANNING_FAILED_ERROR_X` - Failure with error code
- `MAX_REPLAN_EXCEEDED` - Too many failed attempts (>5)

## Configuration

### Main Parameters

```yaml
# CRITICAL: These parameters control which interpolator system is used
enable_collision_checking: true/false        # Path planner collision responsibility
enable_online_collision_checking: true/false # Real-time collision monitoring
use_local_planner: true/false                # Interpolator type selection
```

#### Parameter Explanation:

- **`enable_collision_checking`**: Controls whether the path planner performs collision detection during planning
  - `true`: Path planner ensures collision-free paths (for standard interpolator)
  - `false`: Skip collision checking, rely on local planner (for local interpolator)

- **`enable_online_collision_checking`**: Controls real-time collision monitoring of published paths
  - `true`: Monitor paths continuously and trigger replanning if collisions detected
  - `false`: No monitoring, assume paths remain valid

- **`use_local_planner`**: Indicates which interpolator type the system expects
  - `true`: Optimized for local_trajectory_interpolator with APF
  - `false`: Optimized for standard trajectory_interpolator

#### Parameter Combinations:

**Standard Trajectory Interpolator System**:
```yaml
enable_collision_checking: true           # Path planner does ALL collision work
enable_online_collision_checking: false  # No real-time monitoring needed
use_local_planner: false                 # System expects trajectory_interpolator
```

**Local Trajectory Interpolator System**:
```yaml
enable_collision_checking: false         # Local planner handles detailed collisions
enable_online_collision_checking: true   # Monitor for major obstacles/changes
use_local_planner: true                  # System expects local_trajectory_interpolator
```

#### Integration Examples:

**Standard Interpolator System**:
```bash
# Launch path planner with conservative settings
ros2 launch path_planner path_planner.launch.py \
  config_file:=config/path_planner_params.yaml

# Launch standard trajectory interpolator
ros2 run traj_interp trajectory_interpolator
```

**Local Interpolator System**:
```bash
# Launch path planner with optimized settings  
ros2 launch path_planner path_planner.launch.py \
  config_file:=config/path_planner_flight.yaml

# Launch local trajectory interpolator
ros2 run traj_interp local_trajectory_interpolator
```

## Algorithms

### OMPL Planning
- **RRT*** (default): Optimal Rapidly-exploring Random Tree
- **Space**: SE(3) - Position + orientation
- **State Validation**: FCL collision checking

### Collision Detection
- **FCL Library**: Fast collision detection
- **Octomap Integration**: Voxel-based obstacle representation
- **Real-time Checking**: Continuous path monitoring

### Replan Logic
1. **Collision Detection**: Checks path every 200ms
2. **Collision Point Saving**: Saves collision position for replan
3. **Attempt Limiting**: Maximum 5 attempts per goal
4. **Goal Clearing**: Automatic reset after limit exceeded
## Path Resampling

The system implements automatic resampling for velocity control:

```cpp
// Resampling at 40cm to reduce drone velocities
std::vector<geometry_msgs::msg::PoseStamped> resample_path(path, 0.4);
```

**Benefits**:
- Reduces excessive drone velocities
- Improves trajectory stability
- More precise movement control

## 🛡️ Safety Improvements

### Enhanced Collision Avoidance
The path planner now includes multiple layers of safety:

**Safety Margin System**:
- **Base robot radius**: Physical drone size (0.7m default)
- **Safety margin**: Additional buffer zone (0.3-0.5m)
- **Effective radius**: Combined safety zone (1.0-1.2m total)

**Configuration Examples**:
```yaml
# Conservative (Real Flight)
robot_radius: 0.7
safety_margin: 0.5    # Total: 1.2m safety zone

# Balanced (Simulation)  
robot_radius: 0.7
safety_margin: 0.3    # Total: 1.0m safety zone
```

**Benefits**:
- ✅ **Prevents trajectory deviation collisions**: Accounts for drone drift from planned path
- ✅ **Wind compensation**: Larger safety zone handles wind disturbances
- ✅ **Control lag tolerance**: Buffers for response delays
- ✅ **Sensor uncertainty**: Compensates for localization errors

## System Integration

The path planner integrates with:

1. **babyk_drone_manager**: Receives goals and communicates status
2. **Trajectory Interpolators**: Provides paths for execution (compatible with both)
3. **RTABMap**: Receives octomap for collision detection
4. **RViz**: Visualizes paths and collision markers

## Launch

### Simulation
```bash
ros2 launch path_planner path_planner.launch.py \
  config_file:=config/path_planner_simulation.yaml
```

### Real Flight
```bash
ros2 launch path_planner path_planner.launch.py \
  config_file:=config/path_planner_params.yaml
```

## Troubleshooting

### Planning Failed Error 0
- Check workspace bounds
- Verify octomap: `ros2 topic echo /octomap_binary`
- Reduce `safety_margin` if space is too tight
- Adjust `robot_radius` for tighter spaces

### Collision Despite Clear Path (FIXED)
- ✅ **Safety margin system**: Prevents trajectory deviation collisions
- ✅ **Effective radius**: `robot_radius + safety_margin` used for planning
- ✅ **Configurable safety**: Adjust margin based on environment

**Safety Tuning**:
```yaml
# Tight spaces
safety_margin: 0.2

# Open areas  
safety_margin: 0.5

# Windy conditions
safety_margin: 0.7
```

### Infinite Replan Loop (SOLVED)
- ✅ Implemented 5-attempt limit
- ✅ Automatic goal reset after limit
- ✅ Detailed attempt logging

### High Drone Velocities (SOLVED)  
- ✅ Path resampling every 40cm
- ✅ Automatic waypoint density control
- ✅ Velocity reduction through resampling

### Goals Not Reached
- Verify TF `map` → `base_link`
- Check workspace limits
- Increase `planning_timeout`

## Performance

**Typical Timings**:
- Planning: 1-5 seconds
- Collision Check: <1ms per point
- Replan: 2-8 seconds (depends on complexity)

**Memory Usage**:
- Octomap: ~50-200MB (depends on resolution)
- OMPL Tree: ~10-50MB (depends on space)

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select path_planner
source install/setup.bash
```

## Dependencies

**External Libraries**:
- OMPL (Open Motion Planning Library)
- FCL (Flexible Collision Library)
- Octomap
- Eigen3
- Boost

**ROS 2 Packages**:
- `nav_msgs`, `geometry_msgs`, `visualization_msgs`
- `tf2`, `tf2_ros`, `tf2_geometry_msgs`
- `octomap_msgs`

## Development Notes

- **Thread Safety**: Mutex protection for shared state
- **Memory Management**: Smart pointers for OMPL objects
- **Error Handling**: Comprehensive error codes for debugging
- **Modularity**: Clear separation between planning/execution logic
- OctoMap
- Eigen3
- Boost

## Integration Notes

- Compatible with `traj_interp` for path execution
- Excludes teleop functionality (present in `trajectory_planner`)
- Uses only core path planning methods from `trajectory_planner`
- Optimized for PX4 drones with ENU coordinates
- Modular design allows easy algorithm swapping

## License

This path planning system is part of the Drone Manager package.

### Authors
- **Simone D'Angelo** - simone.dangelo@unina.it
- **Francesca Pagano** - francesca.pagano@unina.it  
- **Vincenzo Scognamiglio** - vincenzo.scognamiglio2@unina.it

**PRISMA LAB** - University of Naples Federico II

---

**Note**: This system is designed for research and development purposes. Always follow local aviation regulations and safety guidelines when operating autonomous vehicles.
