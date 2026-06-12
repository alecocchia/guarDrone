# file: mpc_sim.launch.py

import os
import xml.etree.ElementTree as ET
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import sys
try:
    from drone_mpc_pkg.PX4_model_parser import PX4ModelParser
except ImportError:
    # Fallback se il pacchetto non è sourcato
    sys.path.append('/root/my_ros2_ws/src/drone_mpc_pkg')
    from drone_mpc_pkg.PX4_model_parser import PX4ModelParser

def launch_setup(context, *args, **kwargs):
    # --- RECUPERO ARGOMENTI ---
    model_name = LaunchConfiguration('model').perform(context)
    planner_mode = LaunchConfiguration('planner_mode').perform(context)
    controller = LaunchConfiguration('controller').perform(context)
    MPC_controller = LaunchConfiguration('MPC_controller').perform(context)
    
    # --- CALCOLO AUTOMATICO FISICA ---
    parser = PX4ModelParser()
    auto_mass, auto_inertia, auto_cam, auto_cam_rpy, auto_fov_h, auto_fov_v, auto_fmax, auto_lx, auto_ly, auto_mc = parser.get_px4_model_info(model_name)
    auto_wmin, auto_wmax = parser.get_airframe_params(model_name)
    
    # Se f_max non viene trovato (es. modello non standard), usiamo un fallback basato sulla massa
    if auto_fmax == 0:
        auto_fmax = auto_mass * 9.81 * 2.0

    print(f"\n[AUTO-PHYSICS] Modello: {model_name}")
    print(f"[AUTO-PHYSICS] Massa Totale: {auto_mass:.4f} kg")
    print(f"[AUTO-PHYSICS] Inerzia (Ixx, Iyy, Izz): {auto_inertia}")
    print(f"[AUTO-PHYSICS] Posizione Camera: {auto_cam}")
    print(f"[AUTO-PHYSICS] FOV Camera (H, V): {auto_fov_h:.1f}, {auto_fov_v:.1f} deg")
    print(f"[AUTO-PHYSICS] Spinta Massima (F_max): {auto_fmax:.2f} N")
    print(f"[AUTO-PHYSICS] Range Velocità Motori: [{auto_wmin}, {auto_wmax}] rad/s\n")

    # Percorsi
    mpc_pkg_dir = get_package_share_directory('drone_mpc_pkg')
    bridge_config_file = os.path.join(mpc_pkg_dir, 'config', 'bridge.yaml')
    rviz_config_file = os.path.join(mpc_pkg_dir, 'config', 'rviz_config_file.rviz')

    # Pose e configurazioni
    drone_x = LaunchConfiguration('drone_x')
    drone_y = LaunchConfiguration('drone_y')
    drone_z = LaunchConfiguration('drone_z')
    drone_yaw = LaunchConfiguration('drone_yaw')
    peg_x = LaunchConfiguration('peg_x')
    peg_y = LaunchConfiguration('peg_y')
    peg_z = LaunchConfiguration('peg_z')
    cf = LaunchConfiguration('cf')
    ct = LaunchConfiguration('ct')

    # --- DEFINIZIONE NODI ---
    
    mpc_planner_node = Node(
        package='drone_mpc_pkg',
        executable='MPC_planner_node.py',
        name='MPC_planner_node',
        output='screen', emulate_tty=True,
        parameters=[{
            'use_sim_time': True,
            'control_flag': LaunchConfiguration('MPC_controller'),
            'mass': auto_mass, 
            'ixx': auto_inertia[0], 'iyy': auto_inertia[1], 'izz': auto_inertia[2],
            'cf': cf, 'ct': ct,
            'f_max': auto_fmax,
            'cam_x': auto_cam[0], 'cam_y': auto_cam[1], 'cam_z': auto_cam[2],
            'cam_roll': auto_cam_rpy[0], 'cam_pitch': auto_cam_rpy[1], 'cam_yaw': auto_cam_rpy[2],
            'fov_h': auto_fov_h, 'fov_v': auto_fov_v,
            'start_x': drone_x, 'start_y': drone_y, 'start_z': drone_z,
            'start_yaw': drone_yaw,
            'peg_x': peg_x, 'peg_y': peg_y, 'peg_z': peg_z,
            'w_min': auto_wmin, 'w_max': auto_wmax,
            'arm_l_x': auto_lx, 'arm_l_y': auto_ly, 'moment_const': auto_mc,
        }],
        condition=IfCondition(PythonExpression([f"'{planner_mode}' == '1'"]))
    )

    data_logger = Node(
        package='drone_mpc_pkg',
        executable='logger.py',
        name='data_logger',
        output='screen',
        parameters=[{
            'use_sim_time': True, 
            'save_path': '/tmp/pid_run.npz',
            'mass': auto_mass,
            'cam_x': auto_cam[0], 'cam_y': auto_cam[1], 'cam_z': auto_cam[2], # AUTOMATICO
            'start_x': drone_x, 'start_y': drone_y, 'start_z': drone_z,
            'ft_topic': LaunchConfiguration('peg_ft_topic')
        }],
    )

    peg_planner = Node(
        package='drone_mpc_pkg', executable='offboard_admittance_planner.py', name='peg_trajectory_planner',
        parameters=[{
            'use_sim_time': True,
            'start_x': peg_x, 'start_y': peg_y, 'start_z': peg_z,
            'v_max': 0.5,
            'a_max': 1.0,
            'dt': 0.02,           # 50 Hz
            'px4_ns': 'px4_1',   # Namespace DDS del drone x500_interaction (UXRCE_DDS_NS=px4_1)
            # -- Ammettenza --
            'F_threshold':   LaunchConfiguration('peg_F_threshold'),
            #'adm_mass':      LaunchConfiguration('peg_adm_mass'),
            #'adm_damping':   LaunchConfiguration('peg_adm_damping'),
            #'adm_stiffness': LaunchConfiguration('peg_adm_stiffness'),
            'adm_max_delta': LaunchConfiguration('peg_adm_max_delta'),
            # Topic sensore FT: cambia con l'ordine di spawn (_0 se primo, _1 se secondo).
            # Di default assumiamo x500_interaction_1 perché è il secondo drone spawnato.
            'ft_topic': LaunchConfiguration('peg_ft_topic'),
        }],
        remappings=[
            ('target_pose', '/peg_target_pose'),
            ('offboard_traj_enabled', '/peg_traj_enabled')
        ],
        condition=IfCondition(PythonExpression([f"'{planner_mode}' in ['1', '2']"]))
    )

    camera_planner = Node(
        package='drone_mpc_pkg', executable='offboard_trajectory_planner.py', name='camera_trajectory_planner',
        parameters=[{
            'use_sim_time': True,
            'start_x': drone_x, 'start_y': drone_y, 'start_z': drone_z,
            'v_max': 1.0,
            'a_max': 2.0,
            'px4_ns': '',  # Namespace di root
        }],
        remappings=[
            ('target_pose', '/camera_target_pose'),
            ('offboard_traj_enabled', '/camera_traj_enabled')
        ],
        condition=IfCondition(PythonExpression([f"'{planner_mode}' in ['1', '2']"]))
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config_file}'],
        parameters=[{'use_sim_time': True}]
    )

    human_goal_node = Node(
        package='drone_mpc_pkg',
        executable='human_goal_node.py',
        name='human_goal_node',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('enable_joy'))
    )

    supervisor_node = Node(
        package='drone_mpc_pkg',
        executable='supervisor.py',
        name='supervisor_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'takeoff_alt_1': 4.52+3.0,   # [m] ENU: quota di decollo camera drone
            'takeoff_alt_2': 4.52+3.0,   # [m] ENU: quota di decollo peg drone
            'cam_start_x': drone_x,
            'cam_start_y': drone_y,
            'cam_start_z': drone_z,
            'peg_start_x': peg_x,
            'peg_start_y': peg_y,
            'peg_start_z': peg_z,
        }],
    )

    return [
        supervisor_node,
        peg_planner,
        camera_planner,
        ros_gz_bridge,
        TimerAction(period=10.0, actions=[mpc_planner_node]),
        data_logger,
        human_goal_node,
        TimerAction(period=5.0, actions=[Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config_file], condition=IfCondition(LaunchConfiguration('enable_rviz')))]),
        Node(package='joy', executable='joy_node', parameters=[{'autorepeat_rate': 50.0}], condition=IfCondition(LaunchConfiguration('enable_joy')))
    ]

def generate_launch_description():
    # NOTA: il mondo Gazebo e il drone x500_interaction vengono lanciati
    # separatamente tramite run_my_sim.sh (make px4_sitl), NON da qui.
    # Questo launch si occupa solo dei nodi ROS 2.
    declared_arguments = [
        DeclareLaunchArgument('model', default_value='x500_depth',
                              description='Modello Gazebo del drone MPC (es. x500_depth)'),
        DeclareLaunchArgument('planner_mode', default_value='1',
                              description='1: MPC+planner, 2: solo planner (PX4 position control)'),
        DeclareLaunchArgument('MPC_controller', default_value='1'),
        DeclareLaunchArgument('controller', default_value='2'),
        DeclareLaunchArgument('enable_rviz', default_value='true'),
        DeclareLaunchArgument('enable_joy', default_value='true'),
        DeclareLaunchArgument('drone_x', default_value='0.0'),
        DeclareLaunchArgument('drone_y', default_value='0.0'),
        DeclareLaunchArgument('drone_z', default_value='4.52'),
        DeclareLaunchArgument('drone_yaw', default_value='1.5708'),
        # Posa target per il drone di interazione (passata a peg_planner_node)
        DeclareLaunchArgument('peg_x', default_value='3.0'),
        DeclareLaunchArgument('peg_y', default_value='3.0'),
        DeclareLaunchArgument('peg_z', default_value='4.52'),
        DeclareLaunchArgument('cf', default_value='8.0e-4'),
        DeclareLaunchArgument('ct', default_value='1.0e-5'),
        # -- Parametri ammettenza peg --
        DeclareLaunchArgument('peg_F_threshold',   default_value='0.08',
                              description='[N] Soglia forza per attivare ammettenza'),
        #DeclareLaunchArgument('peg_adm_mass',      default_value='2.0',
        #                      description='[kg] Massa virtuale ammettenza'),
        #DeclareLaunchArgument('peg_adm_damping',   default_value='5.0',
        #                      description='Smorzamento virtuale ammettenza'),
        #DeclareLaunchArgument('peg_adm_stiffness', default_value='80',
        #                      description='Rigidezza virtuale ammettenza (0=puro ammortizzatore)'),
        DeclareLaunchArgument('peg_adm_max_delta', default_value='5.0',
                              description='[m] Saturazione spostamento di ammettenza'),
        DeclareLaunchArgument('peg_ft_topic',
                              default_value='/world/interaction/model/x500_interaction_0/joint/end_eff_sens_joint/force_torque',
                              description='Topic Gazebo del sensore FT sull\'end-effector del peg'),
    ]

    return LaunchDescription(declared_arguments + [
        OpaqueFunction(function=launch_setup)
    ])
