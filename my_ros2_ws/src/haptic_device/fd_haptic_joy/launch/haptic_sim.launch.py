# file: haptic_sim.launch.py
# Launch file per il dispositivo haptic Falcon Force Dimension in simulazione.
# Include: fd_bringup (driver hardware Falcon) + fd_haptic_joy_node (bridge haptic→joy).

# Se il dispositivo Falcon non è collegato, fd_bringup fallirà silenziosamente
# e il resto della simulazione (guardrone, interaction, gcs) continuerà normalmente.

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    fd_bringup_dir = get_package_share_directory('fd_bringup')

    # Driver hardware Falcon Force Dimension
    fd_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fd_bringup_dir, 'launch', 'fd.launch.py')
        )
    )

    # Bridge: converte i dati del Falcon in comandi joy + abilita force feedback
    haptic_joy_node = Node(
        package='fd_haptic_joy',
        executable='fd_haptic_joy_node',
        name='fd_haptic_joy_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'k_spring':   50.0,
            'b_damping':  10.0,
            'v_beta_max':  0.15,   # <-- Velocità di orbita del GuaDrone: ~8.5 deg/s (v_tang = 0.45 m/s a 3m)
            'v_r_max':     0.5,    # <-- Velocità radiale avvicinamento/allontanamento
            'v_z_max':     0.3,    # <-- Velocità quota
            'deadband':    0.008,
            'v_pan_max':   0.7,    # (parametri Peg Drone)
            'v_zc_max':    0.3,
            'v_xc_max':    0.5,
        }]

    )

    return LaunchDescription([
        fd_launch,
        haptic_joy_node,
    ])
