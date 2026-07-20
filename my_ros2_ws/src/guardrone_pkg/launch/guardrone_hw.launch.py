# file: guardrone_hw.launch.py
# Launch file per il GuaDrone su HARDWARE REALE (Drone 1 - MPC + Camera).
# Differenze rispetto a guardrone_sim.launch.py:
#   - Niente PX4ModelParser (parametri fisici passati esplicitamente)
#   - Niente ros_gz_bridge (non c'è Gazebo)
#   - use_sim_time = False (clock reale)
#   - start_x/y/z = 0.0 (MOCAP/OptiTrack gestisce il frame)
#   - peg_x/y/z = 0.0 (MOCAP dà posizioni assolute, niente offset)
#   - RViz disabilitato di default (lanciarlo dal GCS se serve)
#
# Avvia: MPC_planner_node, guarDrone_trajectory_planner.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    print(f"\n[GuaDrone HW] Avvio in hardware. Lettura parametri da YAML.\n")




    # --- PERCORSI ---
    guardrone_pkg_dir = get_package_share_directory('guardrone_pkg')
    rviz_config_file = os.path.join(guardrone_pkg_dir, 'config', 'rviz_config_file.rviz')
    drone_params_file = os.path.join(guardrone_pkg_dir, 'config', 'drone_parameters_hw.yaml')
    mpc_weights_file = os.path.join(guardrone_pkg_dir, 'config', 'mpc_weights.yaml')

    # --- ARGOMENTI POSE ---
    drone_x   = LaunchConfiguration('drone_x')
    drone_y   = LaunchConfiguration('drone_y')
    drone_z   = LaunchConfiguration('drone_z')
    drone_yaw = LaunchConfiguration('drone_yaw')
    peg_x     = LaunchConfiguration('peg_x')
    peg_y     = LaunchConfiguration('peg_y')
    peg_z     = LaunchConfiguration('peg_z')

    # --- NODI ---

    mpc_planner_node = Node(
        package='guardrone_pkg',
        executable='MPC_planner_node.py',
        name='MPC_planner_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            drone_params_file,
            mpc_weights_file,
            {
                'use_sim_time': False,
                'start_x': drone_x, 'start_y': drone_y, 'start_z': drone_z,
                'start_yaw': drone_yaw,
                'peg_x': peg_x, 'peg_y': peg_y, 'peg_z': peg_z,
            }
        ]
    )

    guardrone_trajectory_planner = Node(
        package='guardrone_pkg',
        executable='offboard_trajectory_planner.py',
        name='guarDrone_trajectory_planner',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'start_x': drone_x, 'start_y': drone_y, 'start_z': drone_z,
            'dt': 0.02,   # 50 Hz
            'v_max': 0.5,
            'a_max': 1.0,
            'px4_ns': '',  # Namespace root (istanza PX4 0)
        }],
        remappings=[
            ('target_pose',          '/camera_target_pose'),
            ('offboard_traj_enabled', '/camera_traj_enabled')
        ]
    )

    # --- NODO OPTITRACK (de-commentare in presenza di OptiTrack) ---
    # optitrack_node = Node(
    #     package='optitrack_listener',
    #     executable='optitrack_listener',
    #     name='optitrack_listener',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': False,
    #         'drone_name': 'guardrone', # Nome del rigid body definito su Motive
    #         'px4_ns': '',              # Namespace per mappare su /fmu/in/vehicle_visual_odometry
    #     }]
    # )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('enable_rviz'))
    )

    return [
        guardrone_trajectory_planner,
        # MPC planner parte con un ritardo per dare tempo a PX4 di stabilizzarsi
        TimerAction(period=5.0, actions=[mpc_planner_node]),
        TimerAction(period=3.0, actions=[rviz_node]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enable_rviz', default_value='false',
                              description='Abilitare RViz (disabilitato di default su HW)'),

        # === Pose iniziali (con MOCAP/OptiTrack: default 0.0, il frame è già globale) ===
        DeclareLaunchArgument('drone_x',   default_value='0.0'),
        DeclareLaunchArgument('drone_y',   default_value='0.0'),
        DeclareLaunchArgument('drone_z',   default_value='0.0'),
        DeclareLaunchArgument('drone_yaw', default_value='0.0'),

        # Posizione interaction drone (con MOCAP: default 0.0, niente offset)
        DeclareLaunchArgument('peg_x', default_value='0.0'),
        DeclareLaunchArgument('peg_y', default_value='0.0'),
        DeclareLaunchArgument('peg_z', default_value='0.0'),

        OpaqueFunction(function=launch_setup)
    ])
