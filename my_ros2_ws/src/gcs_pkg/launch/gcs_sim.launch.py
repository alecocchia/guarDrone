# file: gcs_sim.launch.py
# Launch file dedicato alla Ground Control Station in simulazione.
# Avvia: supervisor_node, data_logger.
# Il MicroXRCEAgent è avviato separatamente dal pane 0 della finestra GCS in tmux.

import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

try:
    from utils_pkg.PX4_model_parser import PX4ModelParser
    from utils_pkg.config_loader import load_config
except ImportError:
    sys.path.append('/root/my_ros2_ws/src/utils_pkg')
    sys.path.append(os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'utils_pkg'))
    from utils_pkg.PX4_model_parser import PX4ModelParser
    from utils_pkg.config_loader import load_config



def launch_setup(context, *args, **kwargs):
    # Recupero parametri camera dal modello PX4 (come in guardrone_sim.launch.py)
    model_name = LaunchConfiguration('model').perform(context)
    parser = PX4ModelParser()
    _, _, auto_cam, _, fov_h_deg, fov_v_deg, _, _, _, _ = parser.get_px4_model_info(model_name)

    drone_x = LaunchConfiguration('drone_x')
    drone_y = LaunchConfiguration('drone_y')
    drone_z = LaunchConfiguration('drone_z')
    peg_x   = LaunchConfiguration('peg_x')
    peg_y   = LaunchConfiguration('peg_y')
    peg_z   = LaunchConfiguration('peg_z')
    peg_yaw = LaunchConfiguration('peg_yaw')

    use_fake = LaunchConfiguration('use_fake')

    takeoff_alt_1 = LaunchConfiguration('takeoff_alt_1')
    takeoff_alt_2 = LaunchConfiguration('takeoff_alt_2')

    supervisor_node = Node(
        package='gcs_pkg',
        executable='supervisor.py',
        name='supervisor_node',
        output='screen',
        emulate_tty=True,
        condition=UnlessCondition(use_fake),
        parameters=[{
            'use_sim_time': True,
            'takeoff_alt_1': takeoff_alt_1,
            'takeoff_alt_2': takeoff_alt_2,
            'cam_start_x': drone_x,
            'cam_start_y': drone_y,
            'cam_start_z': drone_z,
            'peg_start_x': peg_x,
            'peg_start_y': peg_y,
            'peg_start_z': peg_z,
            # Offset camera nel body frame
            'cam_offset_x': auto_cam[0],
            'cam_offset_y': auto_cam[1],
            'cam_z_offset': auto_cam[2],
            'peg_z_offset': LaunchConfiguration('peg_z_offset'),
            'peg_delta_x_approach': LaunchConfiguration('peg_delta_x_approach'),
            'peg_delta_y_approach': LaunchConfiguration('peg_delta_y_approach'),
            'peg_delta_z_approach': LaunchConfiguration('peg_delta_z_approach'),
            'peg_delta_x_detachment': LaunchConfiguration('peg_delta_x_detachment'),
            'peg_delta_y_detachment': LaunchConfiguration('peg_delta_y_detachment'),
            'peg_delta_z_detachment': LaunchConfiguration('peg_delta_z_detachment'),
            'fov_h_deg': fov_h_deg,
            'fov_v_deg': fov_v_deg
        }],
    )

    data_logger = Node(
        package='gcs_pkg',
        executable='logger.py',
        name='data_logger',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'save_path': LaunchConfiguration('log_save_path'),
            'start_x': drone_x,
            'start_y': drone_y,
            'start_z': drone_z,
            'cam_x': auto_cam[0], 'cam_y': auto_cam[1], 'cam_z': auto_cam[2],
            'ft_topic': LaunchConfiguration('peg_ft_topic'),
            'fov_h': fov_h_deg,
            'fov_v': fov_v_deg,
            # Drone di interazione: namespace PX4 e offset spawn
            'peg_px4_ns':  'px4_1',
            'peg_start_x': peg_x,
            'peg_start_y': peg_y,
            'peg_start_z': peg_z,
            'record_rosbag': LaunchConfiguration('record_rosbag'),
            'camera_topic':  LaunchConfiguration('camera_topic'),
        }],
    )


    # NODO: FAKE PUBLISHER (usato per testare solo il GuarDrone, simula l'interaction drone e il supervisor)
    fake_publisher_node = Node(
        package='gcs_pkg',
        executable='fake_publisher.py',
        name='fake_publisher_node',
        output='screen',
        condition=IfCondition(use_fake),
        parameters=[{
            'use_sim_time': True,
            'takeoff_alt_1': takeoff_alt_1,
            'guardrone_start_x': drone_x,
            'guardrone_start_y': drone_y,
            'guardrone_start_z': drone_z,
            'peg_start_x': peg_x,
            'peg_start_y': peg_y,
            'peg_start_z': peg_z,
            'peg_start_yaw': peg_yaw,
            'cam_offset_x': auto_cam[0],
            'cam_offset_y': auto_cam[1],
            'cam_offset_z': auto_cam[2],
            'peg_z_offset': LaunchConfiguration('peg_z_offset'),
            'peg_delta_x_approach': LaunchConfiguration('peg_delta_x_approach'),
            'peg_delta_y_approach': LaunchConfiguration('peg_delta_y_approach'),
            'peg_delta_z_approach': LaunchConfiguration('peg_delta_z_approach'),
            'peg_delta_x_detachment': LaunchConfiguration('peg_delta_x_detachment'),
            'peg_delta_y_detachment': LaunchConfiguration('peg_delta_y_detachment'),
            'peg_delta_z_detachment': LaunchConfiguration('peg_delta_z_detachment'),
        }]
    )

    gcs_pkg_dir = get_package_share_directory('gcs_pkg')
    rviz_config_file = os.path.join(gcs_pkg_dir, 'config', 'rviz_config_file.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('enable_rviz'))
    )

    return [supervisor_node, fake_publisher_node, data_logger, rviz_node]


def generate_launch_description():
    cfg = load_config('sim_config.yaml')
    peg_cfg = cfg['peg']
    mission_cfg = cfg['mission']
    pose_cfg = cfg['initial_poses']
    topics_cfg = cfg['topics']

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=str(cfg['model']),
                              description='Modello Gazebo del GuarDrone (per ricavare offset camera)'),
        DeclareLaunchArgument('use_fake', default_value='false',
                              description='Usa fake_publisher invece del supervisor e drone2'),
        DeclareLaunchArgument('enable_rviz', default_value='true',
                              description='Avvia RViz nella GCS'),

        # --- Quote di decollo ---
        DeclareLaunchArgument('takeoff_alt_1', default_value=str(mission_cfg['takeoff_alt_guardrone']),
                              description='Quota ENU decollo camera GuarDrone [m]'),
        DeclareLaunchArgument('takeoff_alt_2', default_value=str(mission_cfg['takeoff_alt_peg']),
                              description='Quota ENU decollo punta peg Interaction Drone [m]'),

        # --- Pose iniziali (devono corrispondere a quelle usate negli altri launch) ---
        DeclareLaunchArgument('drone_x',   default_value=str(pose_cfg['drone_x'])),
        DeclareLaunchArgument('drone_y',   default_value=str(pose_cfg['drone_y'])),
        DeclareLaunchArgument('drone_z',   default_value=str(pose_cfg['drone_z'])),
        DeclareLaunchArgument('peg_x',     default_value=str(pose_cfg['peg_x'])),
        DeclareLaunchArgument('peg_y',     default_value=str(pose_cfg['peg_y'])),
        DeclareLaunchArgument('peg_z',     default_value=str(pose_cfg['peg_z'])),
        DeclareLaunchArgument('peg_yaw',   default_value=str(pose_cfg.get('peg_yaw', 0.0)),
                              description='Yaw iniziale peg/interaction drone [rad]'),
        DeclareLaunchArgument('peg_z_offset', default_value=str(peg_cfg['peg_z_offset']),
                              description='Offset Z della punta end-effector dal CoM del drone [m]'),
        DeclareLaunchArgument('peg_delta_x_approach', default_value=str(peg_cfg['approach']['delta_x']),
                              description='Delta X del peg in APPROACH [m]'),
        DeclareLaunchArgument('peg_delta_y_approach', default_value=str(peg_cfg['approach']['delta_y']),
                              description='Delta Y del peg in APPROACH [m]'),
        DeclareLaunchArgument('peg_delta_z_approach', default_value=str(peg_cfg['approach']['delta_z']),
                              description='Delta Z del peg in APPROACH [m]'),
        DeclareLaunchArgument('peg_delta_x_detachment', default_value=str(peg_cfg['detachment']['delta_x']),
                              description='Delta X del peg in DETACHMENT [m]'),
        DeclareLaunchArgument('peg_delta_y_detachment', default_value=str(peg_cfg['detachment']['delta_y']),
                              description='Delta Y del peg in DETACHMENT [m]'),
        DeclareLaunchArgument('peg_delta_z_detachment', default_value=str(peg_cfg['detachment']['delta_z']),
                              description='Delta Z del peg in DETACHMENT [m]'),
        # --- Parametri logger ---
        DeclareLaunchArgument('peg_ft_topic',
                              default_value=str(topics_cfg['peg_ft_topic']),
                              description='Topic FT del sensore sull\'end-effector (per il logger)'),
        DeclareLaunchArgument('log_save_path', default_value=str(topics_cfg['log_save_path']),
                              description='Percorso file di salvataggio dati'),
        DeclareLaunchArgument('record_rosbag',
                              default_value=str(topics_cfg.get('record_rosbag', False)),
                              description='Se true, avvia automaticamente la registrazione del rosbag'),
        DeclareLaunchArgument('camera_topic',
                              default_value=str(topics_cfg.get('camera_topic', '/camera/camera/color/image_raw/compressed')),
                              description='Topic camera da registrare nel rosbag'),
        OpaqueFunction(function=launch_setup)
    ])


