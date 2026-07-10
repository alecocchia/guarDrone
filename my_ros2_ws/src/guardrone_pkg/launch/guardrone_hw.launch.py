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
    # --- RECUPERO PARAMETRI FISICI (passati dal launch, non dal parser SDF) ---
    mass    = float(LaunchConfiguration('mass').perform(context))
    ixx     = float(LaunchConfiguration('ixx').perform(context))
    iyy     = float(LaunchConfiguration('iyy').perform(context))
    izz     = float(LaunchConfiguration('izz').perform(context))
    f_max   = float(LaunchConfiguration('f_max').perform(context))
    cam_x   = float(LaunchConfiguration('cam_x').perform(context))
    cam_y   = float(LaunchConfiguration('cam_y').perform(context))
    cam_z   = float(LaunchConfiguration('cam_z').perform(context))
    cam_roll  = float(LaunchConfiguration('cam_roll').perform(context))
    cam_pitch = float(LaunchConfiguration('cam_pitch').perform(context))
    cam_yaw   = float(LaunchConfiguration('cam_yaw').perform(context))
    fov_h   = float(LaunchConfiguration('fov_h').perform(context))
    fov_v   = float(LaunchConfiguration('fov_v').perform(context))
    w_min   = float(LaunchConfiguration('w_min').perform(context))
    w_max   = float(LaunchConfiguration('w_max').perform(context))
    arm_l_x = float(LaunchConfiguration('arm_l_x').perform(context))
    arm_l_y = float(LaunchConfiguration('arm_l_y').perform(context))
    moment_const = float(LaunchConfiguration('moment_const').perform(context))

    # Fallback: se f_max non è stato impostato, stima conservativa
    if f_max == 0.0:
        f_max = mass * 9.81 * 2.0

    print(f"\n[GuaDrone HW] Parametri fisici:")
    print(f"[GuaDrone HW] Massa: {mass:.4f} kg | F_max: {f_max:.2f} N")
    print(f"[GuaDrone HW] Inerzia (Ixx,Iyy,Izz): ({ixx}, {iyy}, {izz})")
    print(f"[GuaDrone HW] Camera offset: ({cam_x}, {cam_y}, {cam_z})")
    print(f"[GuaDrone HW] FOV Camera (H,V): {fov_h:.1f}, {fov_v:.1f} deg\n")

    # --- PERCORSI ---
    guardrone_pkg_dir = get_package_share_directory('guardrone_pkg')
    rviz_config_file = os.path.join(guardrone_pkg_dir, 'config', 'rviz_config_file.rviz')

    # --- ARGOMENTI POSE ---
    drone_x   = LaunchConfiguration('drone_x')
    drone_y   = LaunchConfiguration('drone_y')
    drone_z   = LaunchConfiguration('drone_z')
    drone_yaw = LaunchConfiguration('drone_yaw')
    peg_x     = LaunchConfiguration('peg_x')
    peg_y     = LaunchConfiguration('peg_y')
    peg_z     = LaunchConfiguration('peg_z')
    cf        = LaunchConfiguration('cf')
    ct        = LaunchConfiguration('ct')

    # --- NODI ---

    mpc_planner_node = Node(
        package='guardrone_pkg',
        executable='MPC_planner_node.py',
        name='MPC_planner_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': False,
            'control_flag': LaunchConfiguration('MPC_controller'),
            'mass': mass,
            'ixx': ixx, 'iyy': iyy, 'izz': izz,
            'cf': cf, 'ct': ct,
            'f_max': f_max,
            'cam_x': cam_x, 'cam_y': cam_y, 'cam_z': cam_z,
            'cam_roll': cam_roll, 'cam_pitch': cam_pitch, 'cam_yaw': cam_yaw,
            'fov_h': fov_h, 'fov_v': fov_v,
            'start_x': drone_x, 'start_y': drone_y, 'start_z': drone_z,
            'start_yaw': drone_yaw,
            'peg_x': peg_x, 'peg_y': peg_y, 'peg_z': peg_z,
            'w_min': w_min, 'w_max': w_max,
            'arm_l_x': arm_l_x, 'arm_l_y': arm_l_y, 'moment_const': moment_const,
            'return2autonomous': LaunchConfiguration('return2autonomous'),
        }]
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

    fake_publisher_node = Node(
        package='guardrone_pkg',
        executable='fake_publisher.py',
        name='fake_publisher_node',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('enable_rviz'))
    )

    return [
        guardrone_trajectory_planner,
        fake_publisher_node,
        # MPC planner parte con un ritardo per dare tempo a PX4 di stabilizzarsi
        TimerAction(period=5.0, actions=[mpc_planner_node]),
        TimerAction(period=3.0, actions=[rviz_node]),
    ]


def generate_launch_description():
    return LaunchDescription([
        # === Parametri fisici del drone (da misurare sul drone reale) ===
        # TODO: aggiornare con i valori reali misurati
        DeclareLaunchArgument('mass',    default_value='2.064',   description='Massa del drone [kg]'),
        DeclareLaunchArgument('ixx',     default_value='0.0216',  description='Momento di inerzia Ixx [kg·m²]'),
        DeclareLaunchArgument('iyy',     default_value='0.0216',  description='Momento di inerzia Iyy [kg·m²]'),
        DeclareLaunchArgument('izz',     default_value='0.040',   description='Momento di inerzia Izz [kg·m²]'),
        DeclareLaunchArgument('f_max',   default_value='34.0',    description='Spinta massima totale [N]'),
        DeclareLaunchArgument('w_min',   default_value='150.0',   description='Velocità angolare minima motore [rad/s]'),
        DeclareLaunchArgument('w_max',   default_value='1000.0',  description='Velocità angolare massima motore [rad/s]'),
        DeclareLaunchArgument('arm_l_x', default_value='0.174',   description='Braccio motore asse X [m]'),
        DeclareLaunchArgument('arm_l_y', default_value='0.174',   description='Braccio motore asse Y [m]'),
        DeclareLaunchArgument('moment_const', default_value='0.016', description='Costante di momento motore'),

        # === Parametri camera ===
        DeclareLaunchArgument('cam_x',     default_value='0.0',  description='Offset camera X (body) [m]'),
        DeclareLaunchArgument('cam_y',     default_value='0.0',  description='Offset camera Y (body) [m]'),
        DeclareLaunchArgument('cam_z',     default_value='0.0',  description='Offset camera Z (body) [m]'),
        DeclareLaunchArgument('cam_roll',  default_value='0.0',  description='Rotazione camera roll [rad]'),
        DeclareLaunchArgument('cam_pitch', default_value='0.0',  description='Rotazione camera pitch [rad]'),
        DeclareLaunchArgument('cam_yaw',   default_value='0.0',  description='Rotazione camera yaw [rad]'),
        DeclareLaunchArgument('fov_h',     default_value='80.0', description='FOV orizzontale camera [deg]'),
        DeclareLaunchArgument('fov_v',     default_value='60.0', description='FOV verticale camera [deg]'),

        # === Controllo ===
        DeclareLaunchArgument('MPC_controller', default_value='1',
                              description='1 = PX4 thrust/torque integrato, altro = wrench standard'),
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

        # Parametri motore
        DeclareLaunchArgument('cf', default_value='8.0e-4'),
        DeclareLaunchArgument('ct', default_value='1.0e-5'),
        DeclareLaunchArgument('return2autonomous', default_value='False',
                              description='Se True, al rilascio del comando il drone torna alla traiettoria pianificata'),

        OpaqueFunction(function=launch_setup)
    ])
