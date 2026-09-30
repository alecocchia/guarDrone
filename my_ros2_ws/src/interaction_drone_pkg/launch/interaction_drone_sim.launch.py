# file: interaction_drone_sim.launch.py
# Launch file dedicato alla simulazione dell'Interaction Drone (Drone 2 - ammettenza).
# Avvia: offboard_admittance_planner.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Posa iniziale Interaction Drone (NED --> ENU offset di spawn)
        DeclareLaunchArgument('peg_x', default_value='-1.0'),
        DeclareLaunchArgument('peg_y', default_value='-55.0'),
        DeclareLaunchArgument('peg_z', default_value='4.52'),
        # Parametri ammettenza
        DeclareLaunchArgument('F_threshold',       default_value='0.2',
                              description='[N] Soglia forza per attivare ammettenza'),
        DeclareLaunchArgument('peg_adm_max_delta', default_value='10.0',
                              description='[m] Saturazione spostamento di ammettenza'),
        DeclareLaunchArgument('tau_threshold',     default_value='0.01',
                              description='[Nm] Soglia coppia per attivare ammettenza yaw'),
        DeclareLaunchArgument('peg_ft_topic',
                              default_value='/world/interaction/model/x500_interaction/joint/end_eff_sens_joint/force_torque',
                              description='Topic Gazebo del sensore FT sull\'end-effector'),

        # Parametri cinematica traiettoria
        DeclareLaunchArgument('v_takeoff_max',     default_value='0.2',
                              description='[m/s] Velocità massima durante takeoff'),
        DeclareLaunchArgument('a_takeoff_max',     default_value='0.3',
                              description='[m/s^2] Accelerazione massima durante takeoff'),
        DeclareLaunchArgument('v_max',             default_value='0.1',
                              description='[m/s] Velocità massima durante missione'),
        DeclareLaunchArgument('a_max',             default_value='0.2',
                              description='[m/s^2] Accelerazione massima durante missione'),

        Node(
            package='interaction_drone_pkg',
            executable='offboard_admittance_planner.py',
            name='interaction_drone_trajectory_planner',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'use_sim_time': True,
                'start_x': LaunchConfiguration('peg_x'),
                'start_y': LaunchConfiguration('peg_y'),
                'start_z': LaunchConfiguration('peg_z'),
                'v_takeoff_max':     LaunchConfiguration('v_takeoff_max'),
                'a_takeoff_max':     LaunchConfiguration('a_takeoff_max'),
                'v_max':             LaunchConfiguration('v_max'),
                'a_max':             LaunchConfiguration('a_max'),
                'dt': 0.01,         # 100 Hz
                'px4_ns': 'px4_1',  # Namespace DDS del Drone 2 (UXRCE_DDS_NS=px4_1)
                'F_threshold':       LaunchConfiguration('F_threshold'),
                'adm_max_delta':     LaunchConfiguration('peg_adm_max_delta'),
                'tau_threshold':     LaunchConfiguration('tau_threshold'),
                'ft_topic':          LaunchConfiguration('peg_ft_topic'),
            }],
            remappings=[
                ('target_pose',          '/peg_target_pose'),
                ('offboard_traj_enabled', '/peg_traj_enabled')
            ]
        ),
    ])
