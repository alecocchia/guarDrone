# file: interaction_drone_hw.launch.py
# Launch file dedicato all'esecuzione dell'Interaction Drone su HARDWARE REALE (Drone 2 - ammettenza).
# Differenze rispetto a interaction_drone_sim.launch.py:
#   - use_sim_time = False (clock hardware reale)
#   - peg_x/y/z = 0.0 (MOCAP/OptiTrack dà posizioni assolute, niente offset)
#   - peg_ft_topic = '/interaction_drone/force_torque' (topic FT del sensore reale)
#
# Avvia: offboard_admittance_planner.

import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

try:
    from utils_pkg.config_loader import load_config
except ImportError:
    sys.path.append('/root/my_ros2_ws/src/utils_pkg')
    sys.path.append(os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'utils_pkg'))
    from utils_pkg.config_loader import load_config


def generate_launch_description():
    cfg = load_config('hw_config.yaml')
    pose_cfg = cfg['initial_poses']
    topics_cfg = cfg['topics']

    return LaunchDescription([
        # Posa iniziale Interaction Drone (con MOCAP: default 0.0, il frame è già globale)
        DeclareLaunchArgument('peg_x', default_value=str(pose_cfg['peg_x']),
                              description='Posa iniziale peg X [m]'),
        DeclareLaunchArgument('peg_y', default_value=str(pose_cfg['peg_y']),
                              description='Posa iniziale peg Y [m]'),
        DeclareLaunchArgument('peg_z', default_value=str(pose_cfg['peg_z']),
                              description='Posa iniziale peg Z [m]'),

        # Parametri ammettenza
        DeclareLaunchArgument('F_threshold', default_value='0.2',
                              description='[N] Soglia forza per attivare ammettenza'),
        DeclareLaunchArgument('peg_adm_max_delta', default_value='10.0',
                              description='[m] Saturazione spostamento di ammettenza'),
        DeclareLaunchArgument('tau_threshold', default_value='0.002',
                              description='[Nm] Soglia coppia per attivare ammettenza yaw'),
        DeclareLaunchArgument('peg_ft_topic',
                              default_value=str(topics_cfg['peg_ft_topic']),
                              description='Topic FT del sensore hardware'),


        # Parametri cinematica traiettoria
        DeclareLaunchArgument('v_takeoff_max', default_value='0.1',
                              description='[m/s] Velocità massima durante takeoff'),
        DeclareLaunchArgument('a_takeoff_max', default_value='0.2',
                              description='[m/s^2] Accelerazione massima durante takeoff'),
        DeclareLaunchArgument('v_max', default_value='0.2',
                              description='[m/s] Velocità massima durante missione'),
        DeclareLaunchArgument('a_max', default_value='0.3',
                              description='[m/s^2] Accelerazione massima durante missione'),

        Node(
            package='interaction_drone_pkg',
            executable='offboard_admittance_planner.py',
            name='interaction_drone_trajectory_planner',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'use_sim_time': False,
                'start_x': LaunchConfiguration('peg_x'),
                'start_y': LaunchConfiguration('peg_y'),
                'start_z': LaunchConfiguration('peg_z'),
                'v_takeoff_max': LaunchConfiguration('v_takeoff_max'),
                'a_takeoff_max': LaunchConfiguration('a_takeoff_max'),
                'v_max': LaunchConfiguration('v_max'),
                'a_max': LaunchConfiguration('a_max'),
                'dt': 0.01,         # 100 Hz
                'px4_ns': 'px4_1',  # Namespace DDS del Drone 2 (UXRCE_DDS_NS=px4_1)
                'F_threshold': LaunchConfiguration('F_threshold'),
                'adm_max_delta': LaunchConfiguration('peg_adm_max_delta'),
                'tau_threshold': LaunchConfiguration('tau_threshold'),
                'ft_topic': LaunchConfiguration('peg_ft_topic'),
            }],
            remappings=[
                ('target_pose', '/peg_target_pose'),
                ('offboard_traj_enabled', '/peg_traj_enabled')
            ]
        ),
    ])
