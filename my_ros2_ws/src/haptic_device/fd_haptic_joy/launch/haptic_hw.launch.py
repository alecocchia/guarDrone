# file: haptic_hw.launch.py
# Launch file per il dispositivo haptic Falcon Force Dimension in esperimenti HARDWARE REALI.
# Include: fd_bringup (driver hardware Falcon via USB/DHD) + fd_haptic_joy_node (teleoperazione + force feedback).

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    fd_bringup_dir = get_package_share_directory('fd_bringup')

    # Argomenti configurabili
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    fov_h = LaunchConfiguration('fov_h')
    fov_v = LaunchConfiguration('fov_v')
    r_min_safety = LaunchConfiguration('r_min_safety')
    v_r_max = LaunchConfiguration('v_r_max')
    v_beta_max = LaunchConfiguration('v_beta_max')
    v_z_max = LaunchConfiguration('v_z_max')

    # Driver hardware Falcon Force Dimension (dialoga col dispositivo USB reale via ros2_control)
    fd_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fd_bringup_dir, 'launch', 'fd.launch.py')
        ),
        launch_arguments={
            'use_fake_hardware': use_fake_hardware,
        }.items()
    )

    # Bridge: converte le letture del Falcon in comandi PoV/teleop e gestisce il force feedback
    haptic_joy_node = Node(
        package='fd_haptic_joy',
        executable='fd_haptic_joy_node',
        name='fd_haptic_joy_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': use_sim_time,
            # Parametri ritorno elastico e smorzamento aptico
            'k_spring': LaunchConfiguration('k_spring'),
            'b_damping': LaunchConfiguration('b_damping'),
            'deadband': LaunchConfiguration('deadband'),
            'max_force': LaunchConfiguration('max_force'),
            # Limiti di velocità per il GuarDrone (mantenute conservative per l'arena reale)
            'v_beta_max': v_beta_max,
            'v_r_max': v_r_max,
            'v_z_max': v_z_max,
            # Limiti di velocità per il Peg Drone (modalità teleoperazione)
            'v_pan_max': LaunchConfiguration('v_pan_max'),
            'v_zc_max': LaunchConfiguration('v_zc_max'),
            'v_xc_max': LaunchConfiguration('v_xc_max'),
            # Parametri ottici della telecamera reale (per il force feedback dei vincoli di visuale)
            'fov_h': fov_h,
            'fov_v': fov_v,
            # Sicurezza arena reale: distanza minima consentita e guadagni del campo potenziale
            'r_min_safety': r_min_safety,
            'k_repulsive': LaunchConfiguration('k_repulsive'),
            'max_repulsive_force': LaunchConfiguration('max_repulsive_force'),
        }]
    )

    return LaunchDescription([
        # --- Configurazione base ---
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Usa il clock di sistema (False per hardware reale)'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false',
                              description='False per collegarsi al dispositivo USB fisico'),

        # --- FoV telecamera reale (es. RealSense D435i Depth: 87x58 deg, RGB: 69x42 deg) ---
        DeclareLaunchArgument('fov_h', default_value='87.0',
                              description='FoV orizzontale reale [deg] (default 87.0 per RealSense D435i Depth)'),
        DeclareLaunchArgument('fov_v', default_value='58.0',
                              description='FoV verticale reale [deg] (default 58.0 per RealSense D435i Depth)'),

        # --- Limiti di sicurezza e distanze ---
        DeclareLaunchArgument('r_min_safety', default_value='1.5',
                              description='Distanza minima di sicurezza dal target [m]'),
        DeclareLaunchArgument('k_repulsive', default_value='1.0',
                              description='Guadagno forza repulsiva barriera'),
        DeclareLaunchArgument('max_repulsive_force', default_value='15.0',
                              description='Forza repulsiva massima consentita [N]'),

        # --- Velocità teleoperazione (GuaDrone) ---
        DeclareLaunchArgument('v_beta_max', default_value='0.3',
                              description='Velocità max azimut GuarDrone [rad/s]'),
        DeclareLaunchArgument('v_r_max', default_value='0.3',
                              description='Velocità max radiale GuarDrone [m/s]'),
        DeclareLaunchArgument('v_z_max', default_value='0.2',
                              description='Velocità max quota GuarDrone [m/s]'),

        # --- Velocità teleoperazione (Peg Drone) ---
        DeclareLaunchArgument('v_xc_max', default_value='0.5',
                              description='Velocità traslazione Peg [m/s]'),
        DeclareLaunchArgument('v_zc_max', default_value='0.3',
                              description='Velocità quota Peg [m/s]'),
        DeclareLaunchArgument('v_pan_max', default_value='0.5',
                              description='Velocità yaw Peg [rad/s]'),

        # --- Proprietà meccaniche virtuali Falcon ---
        DeclareLaunchArgument('k_spring', default_value='50.0',
                              description='Rigidezza molla di centratura virtuale [N/m]'),
        DeclareLaunchArgument('b_damping', default_value='10.0',
                              description='Coefficiente smorzamento viscoso virtuale'),
        DeclareLaunchArgument('deadband', default_value='0.008',
                              description='Deadband joystick [m]'),
        DeclareLaunchArgument('max_force', default_value='15.0',
                              description='Saturazione forza hardware Falcon [N]'),

        fd_launch,
        haptic_joy_node,
    ])
