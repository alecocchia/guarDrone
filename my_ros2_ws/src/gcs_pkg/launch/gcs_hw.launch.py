# file: gcs_hw.launch.py
# Launch file dedicato alla Ground Control Station per test in HARDWARE.
# Avvia: data_logger e, a scelta, supervisor_node OPPURE fake_publisher_node.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    # Argomenti
    drone_x = LaunchConfiguration('drone_x')
    drone_y = LaunchConfiguration('drone_y')
    drone_z = LaunchConfiguration('drone_z')
    peg_x   = LaunchConfiguration('peg_x')
    peg_y   = LaunchConfiguration('peg_y')
    peg_z   = LaunchConfiguration('peg_z')
    use_fake = LaunchConfiguration('use_fake_supervisor')
    
    # Parametri camera HW (di solito passati al drone, il logger GCS li usa solo per referenza)
    cam_x = LaunchConfiguration('cam_x')
    cam_y = LaunchConfiguration('cam_y')
    cam_z = LaunchConfiguration('cam_z')

    # NODO: SUPERVISOR (usato per missioni multi-drone reali)
    supervisor_node = Node(
        package='gcs_pkg',
        executable='supervisor.py',
        name='supervisor_node',
        output='screen',
        emulate_tty=True,
        condition=UnlessCondition(use_fake),
        parameters=[{
            'use_sim_time': False,
            'takeoff_alt_1': 1.5,   # [m] Quota di decollo reale per GuaDrone
            'takeoff_alt_2': 1.5,   # [m] Quota di decollo reale per Interaction Drone
            'cam_start_x': drone_x,
            'cam_start_y': drone_y,
            'cam_start_z': drone_z,
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
            'use_sim_time': False,
            'takeoff_alt_1': 1.5,   # [m] Quota di decollo reale per GuaDrone
            'cam_start_x': drone_x,
            'cam_start_y': drone_y,
            'cam_start_z': drone_z,
            'peg_start_x': peg_x,
            'peg_start_y': peg_y,
            'peg_start_z': peg_z,
        }],
    )

    # NODO: LOGGER
    data_logger = Node(
        package='gcs_pkg',
        executable='logger.py',
        name='data_logger',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'save_path': LaunchConfiguration('log_save_path'),
            'start_x': drone_x,
            'start_y': drone_y,
            'start_z': drone_z,
            'cam_x': cam_x, 'cam_y': cam_y, 'cam_z': cam_z,
            'ft_topic': LaunchConfiguration('peg_ft_topic'),
            # Drone di interazione: namespace PX4 e offset spawn
            'peg_px4_ns':  'px4_1',
            'peg_start_x': peg_x,
            'peg_start_y': peg_y,
            'peg_start_z': peg_z,
        }],
    )

    return [supervisor_node, fake_publisher_node, data_logger]


def generate_launch_description():
    return LaunchDescription([
        # --- Modalità Esecuzione ---
        DeclareLaunchArgument('use_fake_supervisor', default_value='false',
                              description='Se true, avvia fake_publisher invece del supervisor vero per testare il drone singolo'),
        
        # --- Pose iniziali (con MOCAP: default 0.0, il frame è già globale) ---
        DeclareLaunchArgument('drone_x',   default_value='0.0'),
        DeclareLaunchArgument('drone_y',   default_value='0.0'),
        DeclareLaunchArgument('drone_z',   default_value='0.0'),
        DeclareLaunchArgument('peg_x',     default_value='0.0'),
        DeclareLaunchArgument('peg_y',     default_value='0.0'),
        DeclareLaunchArgument('peg_z',     default_value='0.0'),
        
        # --- Offset Camera HW (usati per log e target reference se serve) ---
        DeclareLaunchArgument('cam_x',     default_value='0.105'),
        DeclareLaunchArgument('cam_y',     default_value='0.0'),
        DeclareLaunchArgument('cam_z',     default_value='0.02'),

        # --- Parametri logger ---
        DeclareLaunchArgument('peg_ft_topic',
                              default_value='/interaction_drone/force_torque',
                              description='Topic FT del sensore hardware'),
        DeclareLaunchArgument('log_save_path', default_value='/tmp/hw_run.npz',
                              description='Percorso file di salvataggio dati log'),
        OpaqueFunction(function=launch_setup)
    ])
