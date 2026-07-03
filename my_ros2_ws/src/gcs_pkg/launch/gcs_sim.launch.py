# file: gcs_sim.launch.py
# Launch file dedicato alla Ground Control Station in simulazione.
# Avvia: supervisor_node, data_logger.
# Il MicroXRCEAgent è avviato separatamente dal pane 0 della finestra GCS in tmux.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
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

        # --- Supervisor: coordina decollo e missione di entrambi i droni ---
        Node(
            package='gcs_pkg',
            executable='supervisor.py',
            name='supervisor_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'use_sim_time': True,
                'takeoff_alt_1': 4.52 + 3.0,   # [m] ENU: quota decollo GuaDrone
                'takeoff_alt_2': 4.52 + 3.0,   # [m] ENU: quota decollo Interaction Drone
                'cam_start_x': LaunchConfiguration('drone_x'),
                'cam_start_y': LaunchConfiguration('drone_y'),
                'cam_start_z': LaunchConfiguration('drone_z'),
                'peg_start_x': LaunchConfiguration('peg_x'),
                'peg_start_y': LaunchConfiguration('peg_y'),
                'peg_start_z': LaunchConfiguration('peg_z'),
            }],
        ),

        # --- Data Logger: registra dati da entrambi i droni ---
        Node(
            package='gcs_pkg',
            executable='logger.py',
            name='data_logger',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'save_path': LaunchConfiguration('log_save_path'),
                'start_x': LaunchConfiguration('drone_x'),
                'start_y': LaunchConfiguration('drone_y'),
                'start_z': LaunchConfiguration('drone_z'),
                'ft_topic': LaunchConfiguration('peg_ft_topic'),
                # Drone di interazione: namespace PX4 e offset spawn
                'peg_px4_ns':  'px4_1',
                'peg_start_x': LaunchConfiguration('peg_x'),
                'peg_start_y': LaunchConfiguration('peg_y'),
                'peg_start_z': LaunchConfiguration('peg_z'),
            }],
        ),
    ])
