import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Directories
    mpc_pkg_dir = get_package_share_directory('drone_mpc_pkg')
    fd_bringup_dir = get_package_share_directory('fd_bringup')
    
    # 1. Include MPC Sim Launch (Gazebo + MPC Planner)
    # Disabilitiamo il joy node originale di mpc_sim per non avere conflitti
    mpc_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(mpc_pkg_dir, 'launch', 'mpc_sim.launch.py')),
        launch_arguments={
            'enable_joy': 'false', # Disabilitiamo il joystick standard
            'enable_rviz': 'true',
            'model': LaunchConfiguration('model'),
            'drone_x': LaunchConfiguration('drone_x'),
            'drone_y': LaunchConfiguration('drone_y'),
            'drone_z': LaunchConfiguration('drone_z'),
            'drone_yaw': LaunchConfiguration('drone_yaw'),
            'peg_x': LaunchConfiguration('peg_x'),
            'peg_y': LaunchConfiguration('peg_y'),
            'peg_z': LaunchConfiguration('peg_z')
        }.items()
    )

    # 2. Include Falcon Hardware Bringup
    fd_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(fd_bringup_dir, 'launch', 'fd.launch.py'))
    )

    # 3. Haptic Joy Bridge Node (C++)
    haptic_joy_node = Node(
        package='fd_haptic_joy',
        executable='fd_haptic_joy_node',
        name='fd_haptic_joy_node',
        output='screen',
        parameters=[{
            'k_spring': 50.0,
            'b_damping': 10.0,
            'v_pan_max': 0.5,
            'v_zc_max': 0.5,
            'v_xc_max': 0.8,
            'deadband': 0.005
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='x500_depth'),
        DeclareLaunchArgument('drone_x', default_value='-17.0'),
        DeclareLaunchArgument('drone_y', default_value='-35.0'),
        DeclareLaunchArgument('drone_z', default_value='0.0'),
        DeclareLaunchArgument('drone_yaw', default_value='0.0'),
        DeclareLaunchArgument('peg_x', default_value='-15.0'),
        DeclareLaunchArgument('peg_y', default_value='-37.0'),
        DeclareLaunchArgument('peg_z', default_value='0.0'),
        mpc_sim_launch,
        fd_launch,
        haptic_joy_node
    ])
