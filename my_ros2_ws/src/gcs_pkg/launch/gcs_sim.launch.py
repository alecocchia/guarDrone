# file: gcs_sim.launch.py
# Launch file dedicato alla Ground Control Station in simulazione.
# Avvia: supervisor_node, data_logger.
# Il MicroXRCEAgent è avviato separatamente dal pane 0 della finestra GCS in tmux.

import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

try:
    from utils_pkg.PX4_model_parser import PX4ModelParser
except ImportError:
    sys.path.append('/root/my_ros2_ws/src/utils_pkg')
    from utils_pkg.PX4_model_parser import PX4ModelParser


def launch_setup(context, *args, **kwargs):
    # Recupero parametri camera dal modello PX4 (come in guardrone_sim.launch.py)
    model_name = LaunchConfiguration('model').perform(context)
    parser = PX4ModelParser()
    _, _, auto_cam, _, _, _, _, _, _, _ = parser.get_px4_model_info(model_name)

    drone_x = LaunchConfiguration('drone_x')
    drone_y = LaunchConfiguration('drone_y')
    drone_z = LaunchConfiguration('drone_z')
    peg_x   = LaunchConfiguration('peg_x')
    peg_y   = LaunchConfiguration('peg_y')
    peg_z   = LaunchConfiguration('peg_z')

    use_fake = LaunchConfiguration('use_fake')

    supervisor_node = Node(
        package='gcs_pkg',
        executable='supervisor.py',
        name='supervisor_node',
        output='screen',
        emulate_tty=True,
        condition=UnlessCondition(use_fake),
        parameters=[{
            'use_sim_time': True,
            'takeoff_alt_1': 4.52 + 3.0,   # [m] ENU: quota decollo GuaDrone
            'takeoff_alt_2': 4.52 + 3.0,   # [m] ENU: quota decollo Interaction Drone
            'cam_start_x': drone_x,
            'cam_start_y': drone_y,
            'cam_start_z': drone_z,
            'peg_start_x': peg_x,
            'peg_start_y': peg_y,
            'peg_start_z': peg_z,
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
            # Drone di interazione: namespace PX4 e offset spawn
            'peg_px4_ns':  'px4_1',
            'peg_start_x': peg_x,
            'peg_start_y': peg_y,
            'peg_start_z': peg_z,
        }],
    )

    # NODO: FAKE PUBLISHER (usato per testare solo il GuaDrone, simula l'interaction drone e il supervisor)
    fake_publisher_node = Node(
        package='gcs_pkg',
        executable='fake_publisher.py',
        name='fake_publisher_node',
        output='screen',
        condition=IfCondition(use_fake),
        parameters=[{
            'use_sim_time': True,
            'takeoff_alt_1': 4.52 + 3.0,
            'cam_start_x': drone_x,
            'cam_start_y': drone_y,
            'cam_start_z': drone_z,
            'peg_start_x': peg_x,
            'peg_start_y': peg_y,
            'peg_start_z': peg_z,
        }]
    )

    return [supervisor_node, fake_publisher_node, data_logger]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='x500_depth',
                              description='Modello Gazebo del GuaDrone (per ricavare offset camera)'),
        DeclareLaunchArgument('use_fake', default_value='false',
                              description='Usa fake_publisher invece del supervisor e drone2'),
        # --- Pose iniziali (devono corrispondere a quelle usate negli altri launch) ---
        DeclareLaunchArgument('drone_x',   default_value='-4.0'),
        DeclareLaunchArgument('drone_y',   default_value='-53.0'),
        DeclareLaunchArgument('drone_z',   default_value='4.52'),
        DeclareLaunchArgument('peg_x',     default_value='-1.0'),
        DeclareLaunchArgument('peg_y',     default_value='-55.0'),
        DeclareLaunchArgument('peg_z',     default_value='4.52'),
        # --- Parametri logger ---
        DeclareLaunchArgument('peg_ft_topic',
                              default_value='/world/interaction/model/x500_interaction/joint/end_eff_sens_joint/force_torque',
                              description='Topic FT del sensore sull\'end-effector (per il logger)'),
        DeclareLaunchArgument('log_save_path', default_value='/tmp/sim_run.npz',
                              description='Percorso file di salvataggio dati'),
        OpaqueFunction(function=launch_setup)
    ])
