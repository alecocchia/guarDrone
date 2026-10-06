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
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

try:
    from utils_pkg.config_loader import load_config
except ImportError:
    sys.path.append('/root/my_ros2_ws/src/utils_pkg')
    sys.path.append(os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'utils_pkg'))
    from utils_pkg.config_loader import load_config


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

    # --- ARGOMENTI POSE ---
    drone_x   = LaunchConfiguration('drone_x')
    drone_y   = LaunchConfiguration('drone_y')
    drone_z   = LaunchConfiguration('drone_z')
    drone_yaw = LaunchConfiguration('drone_yaw')
    peg_x     = LaunchConfiguration('peg_x')
    peg_y     = LaunchConfiguration('peg_y')
    peg_z     = LaunchConfiguration('peg_z')
    cf         = LaunchConfiguration('cf')
    ct         = LaunchConfiguration('ct')
    controller = LaunchConfiguration('controller')

    # --- NODI ---

    mpc_planner_node = Node(
        package='guardrone_pkg',
        executable='MPC_planner_node.py',
        name='MPC_planner_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': False,
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
            'use_mbe': True,
            'controller': controller,
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
            'v_max': 0.3,
            'a_max': 0.6,
            'px4_ns': '',  # Namespace root (istanza PX4 0)
        }],
        remappings=[
            ('target_pose',          '/camera_target_pose'),
            ('offboard_traj_enabled', '/camera_traj_enabled')
        ]
    )

    return [
        guardrone_trajectory_planner,
        # MPC planner parte con un ritardo per dare tempo a PX4 di stabilizzarsi
        TimerAction(period=5.0, actions=[mpc_planner_node]),
    ]


def generate_launch_description():
    cfg = load_config('hw_config.yaml')
    cam_cfg = cfg['camera']
    phys_cfg = cfg['guardrone_physics']
    pose_cfg = cfg['initial_poses']

    return LaunchDescription([
        # === Parametri fisici del drone (misurati sul drone reale) ===
        DeclareLaunchArgument('mass',         default_value=str(phys_cfg['mass']),         description='Massa del drone [kg]'),
        DeclareLaunchArgument('ixx',          default_value=str(phys_cfg['ixx']),          description='Momento di inerzia Ixx [kg·m²]'),
        DeclareLaunchArgument('iyy',          default_value=str(phys_cfg['iyy']),          description='Momento di inerzia Iyy [kg·m²]'),
        DeclareLaunchArgument('izz',          default_value=str(phys_cfg['izz']),          description='Momento di inerzia Izz [kg·m²]'),
        DeclareLaunchArgument('f_max',        default_value=str(phys_cfg['f_max']),        description='Spinta massima totale [N]'),
        DeclareLaunchArgument('w_min',        default_value=str(phys_cfg['w_min']),        description='Velocità angolare minima motore [rad/s]'),
        DeclareLaunchArgument('w_max',        default_value=str(phys_cfg['w_max']),        description='Velocità angolare massima motore [rad/s]'),
        DeclareLaunchArgument('arm_l_x',      default_value=str(phys_cfg['arm_l_x']),      description='Braccio motore asse X [m]'),
        DeclareLaunchArgument('arm_l_y',      default_value=str(phys_cfg['arm_l_y']),      description='Braccio motore asse Y [m]'),
        DeclareLaunchArgument('moment_const', default_value=str(phys_cfg['moment_const']), description='Costante di momento motore'),

        # === Parametri camera ===
        DeclareLaunchArgument('cam_x',     default_value=str(cam_cfg['cam_x']),     description='Offset camera X (body) [m]'),
        DeclareLaunchArgument('cam_y',     default_value=str(cam_cfg['cam_y']),     description='Offset camera Y (body) [m]'),
        DeclareLaunchArgument('cam_z',     default_value=str(cam_cfg['cam_z']),     description='Offset camera Z (body) [m]'),
        DeclareLaunchArgument('cam_roll',  default_value=str(cam_cfg['cam_roll']),  description='Rotazione camera roll [rad]'),
        DeclareLaunchArgument('cam_pitch', default_value=str(cam_cfg['cam_pitch']), description='Rotazione camera pitch [rad]'),
        DeclareLaunchArgument('cam_yaw',   default_value=str(cam_cfg['cam_yaw']),   description='Rotazione camera yaw [rad]'),
        DeclareLaunchArgument('fov_h',     default_value=str(cam_cfg['fov_h_deg']), description='FOV orizzontale camera [deg]'),
        DeclareLaunchArgument('fov_v',     default_value=str(cam_cfg['fov_v_deg']), description='FOV verticale camera [deg]'),

        # === Controllo ===
        DeclareLaunchArgument('controller', default_value=str(phys_cfg['controller']),
                              description='1: MPC come controllore (spinta e coppie), 0: MPC come planner (setpoint pos/vel)'),

        # === Pose iniziali (con MOCAP/OptiTrack: default 0.0, il frame è già globale) ===
        DeclareLaunchArgument('drone_x',   default_value=str(pose_cfg['drone_x'])),
        DeclareLaunchArgument('drone_y',   default_value=str(pose_cfg['drone_y'])),
        DeclareLaunchArgument('drone_z',   default_value=str(pose_cfg['drone_z'])),
        DeclareLaunchArgument('drone_yaw', default_value=str(pose_cfg['drone_yaw'])),

        # Posizione interaction drone (con MOCAP: default 0.0, niente offset)
        DeclareLaunchArgument('peg_x', default_value=str(pose_cfg['peg_x'])),
        DeclareLaunchArgument('peg_y', default_value=str(pose_cfg['peg_y'])),
        DeclareLaunchArgument('peg_z', default_value=str(pose_cfg['peg_z'])),

        # Parametri motore
        DeclareLaunchArgument('cf', default_value=str(phys_cfg['cf'])),
        DeclareLaunchArgument('ct', default_value=str(phys_cfg['ct'])),

        OpaqueFunction(function=launch_setup)
    ])

