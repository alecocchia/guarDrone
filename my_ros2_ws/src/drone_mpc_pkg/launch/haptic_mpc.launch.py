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
            'enable_rviz': 'true'
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
            'k_spring': 40.0,
            'v_pan_max': 0.5,
            'v_zc_max': 0.5,
            'v_xc_max': 1.0,
            'deadband': 0.005
        }]
    )

    return LaunchDescription([
        mpc_sim_launch,
        fd_launch,
        haptic_joy_node
    ])
