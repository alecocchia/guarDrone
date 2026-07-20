# file: guardrone_sim.launch.py
# Launch file dedicato alla simulazione del GuarDrone (Drone 1 - MPC + Camera).
# Avvia: MPC_planner_node, guarDrone_trajectory_planner, ros_gz_bridge, rviz2.

import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

try:
    from utils_pkg.PX4_model_parser import PX4ModelParser
except ImportError:
    sys.path.append('/root/my_ros2_ws/src/utils_pkg')
    from utils_pkg.PX4_model_parser import PX4ModelParser


def launch_setup(context, *args, **kwargs):
    # --- RECUPERO ARGOMENTI ---
    model_name    = LaunchConfiguration('model').perform(context)
    # --- CALCOLO AUTOMATICO FISICA DAL MODELLO PX4 ---
    parser = PX4ModelParser()
    auto_mass, auto_inertia, auto_cam, auto_cam_rpy, auto_fov_h, auto_fov_v, \
        auto_fmax, auto_lx, auto_ly, auto_mc = parser.get_px4_model_info(model_name)
    auto_wmin, auto_wmax = parser.get_airframe_params(model_name)

    if auto_fmax == 0:
        auto_fmax = auto_mass * 9.81 * 2.0

    print(f"\n[GuarDrone SIM] Modello: {model_name}")
    print(f"[GuarDrone SIM] Massa: {auto_mass:.4f} kg | F_max: {auto_fmax:.2f} N")
    print(f"[GuarDrone SIM] Inerzia (Ixx,Iyy,Izz): {auto_inertia}")
    print(f"[GuarDrone SIM] FOV Camera (H,V): {auto_fov_h:.1f}, {auto_fov_v:.1f} deg\n")

    # --- PERCORSI ---
    guardrone_pkg_dir = get_package_share_directory('guardrone_pkg')
    # Il bridge.yaml e rviz vivono in guardrone_pkg
    bridge_config_file = os.path.join(guardrone_pkg_dir, 'config', 'bridge.yaml')
    rviz_config_file   = os.path.join(guardrone_pkg_dir, 'config', 'rviz_config_file.rviz')
    drone_params_file = os.path.join(guardrone_pkg_dir, 'config', 'drone_parameters_sim.yaml')
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
                'use_sim_time': True,
                'mass': auto_mass,
                'ixx': auto_inertia[0], 'iyy': auto_inertia[1], 'izz': auto_inertia[2],
                'f_max': auto_fmax,
                'cam_x': auto_cam[0], 'cam_y': auto_cam[1], 'cam_z': auto_cam[2],
                'cam_roll': auto_cam_rpy[0], 'cam_pitch': auto_cam_rpy[1], 'cam_yaw': auto_cam_rpy[2],
                'fov_h': auto_fov_h, 'fov_v': auto_fov_v,
                'start_x': drone_x, 'start_y': drone_y, 'start_z': drone_z,
                'start_yaw': drone_yaw,
                'peg_x': peg_x, 'peg_y': peg_y, 'peg_z': peg_z,
                'w_min': auto_wmin, 'w_max': auto_wmax,
                'arm_l_x': auto_lx, 'arm_l_y': auto_ly, 'moment_const': auto_mc,
            }
        ]
    )

    guardrone_trajectory_planner = Node(
        package='guardrone_pkg',
        executable='offboard_trajectory_planner.py',
        name='guarDrone_trajectory_planner',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'start_x': drone_x, 'start_y': drone_y, 'start_z': drone_z,
            'dt': 0.02,   # 50 Hz
            'v_max': 0.5,
            'a_max': 1.0,
            'px4_ns': '',  # Namespace root (Drone 1 = istanza 0)
        }],
        remappings=[
            ('target_pose',          '/camera_target_pose'),
            ('offboard_traj_enabled', '/camera_traj_enabled')
        ]
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config_file}'],
        parameters=[{'use_sim_time': True}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('enable_rviz'))
    )

    return [
        ros_gz_bridge,
        guardrone_trajectory_planner,
        # MPC planner parte con un ritardo per dare tempo a PX4 e bridge di stabilizzarsi
        TimerAction(period=10.0, actions=[mpc_planner_node]),
        TimerAction(period=5.0,  actions=[rviz_node]),
    ]


def generate_launch_description():
    return LaunchDescription([
        # Modello PX4/Gazebo del GuarDrone
        DeclareLaunchArgument('model',          default_value='x500_depth',
                              description='Modello Gazebo del GuarDrone (es. x500_depth)'),
        DeclareLaunchArgument('controller',     default_value='2'),
        DeclareLaunchArgument('enable_rviz',    default_value='true'),
        # Posa iniziale GuarDrone
        DeclareLaunchArgument('drone_x',   default_value='-4.0'),
        DeclareLaunchArgument('drone_y',   default_value='-53.0'),
        DeclareLaunchArgument('drone_z',   default_value='4.52'),
        DeclareLaunchArgument('drone_yaw', default_value='0.0'),
        # Posa iniziale Interaction Drone (usata dall'MPC come target peg)
        DeclareLaunchArgument('peg_x', default_value='-1.0'),
        DeclareLaunchArgument('peg_y', default_value='-55.0'),
        DeclareLaunchArgument('peg_z', default_value='4.52'),
        OpaqueFunction(function=launch_setup)
    ])
