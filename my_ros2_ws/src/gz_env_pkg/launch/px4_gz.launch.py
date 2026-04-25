import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():

    # --- DICHIARAZIONE ARGOMENTI PER IL PEG ---
    peg_x_arg = DeclareLaunchArgument('peg_x', default_value='2.0')
    peg_y_arg = DeclareLaunchArgument('peg_y', default_value='0.0')
    peg_z_arg = DeclareLaunchArgument('peg_z', default_value='0.0')

    # Recupero delle configurazioni
    peg_x = LaunchConfiguration('peg_x')
    peg_y = LaunchConfiguration('peg_y')
    peg_z = LaunchConfiguration('peg_z')

    # Argument to choose the world from the command line
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value='peg_hole.sdf',
        description='Name of the SDF world file to load'
    )

    world = LaunchConfiguration('world')

    # Percorsi
    pkg_share_dir = get_package_share_directory('gz_env_pkg')
    px4_worlds_path = '/root/PX4-Autopilot/Tools/simulation/gz/worlds'

    # Logica per decidere il percorso del mondo (se nel pacchetto o in PX4)
    # Usiamo PythonExpression per gestire il percorso dinamicamente
    full_world_path = PythonExpression([
        "os.path.join('", pkg_share_dir, "', 'worlds', '", world, "') if os.path.exists(os.path.join('", 
        pkg_share_dir, "', 'worlds', '", world, "')) else os.path.join('", px4_worlds_path, "', '", world, "')"
    ])

    # Command to launch Gazebo Sim with the selected world
    gazebo_cmd = ExecuteProcess(
        cmd=['gz', 'sim', '-r', full_world_path],
        output='screen'
    )

    # Recupero del percorso del modello
    # Ottiene il percorso della cartella 'share' del tuo pacchetto compilato
    pkg_share_dir = get_package_share_directory('gz_env_pkg')
    
    # Costruisce il percorso assoluto fino al file model.sdf del peg
    peg_model_path = os.path.join(pkg_share_dir, 'models', 'peg_drone', 'model.sdf')


    # Nodo per lo spawn del peg
    # Esegue ros_gz_sim per inserire l'oggetto nell'ambiente
    spawn_peg_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', PythonExpression(["'", world, "'.split('.')[0]"]),    # Estrae il nome dal file .sdf
            '-name', 'my_peg_drone', # Il nome univoco dell'oggetto nella simulazione
            '-file', peg_model_path,
            '-x', peg_x,            # Posizione X
            '-y', peg_y,            # Posizione Y
            '-z', peg_z             # Posizione Z
        ],
        output='screen'
    )

    # Comando per far girare le eliche (solo effetto visivo)
    spin_propellers_cmd = ExecuteProcess(
        cmd=['gz topic -t /model/my_peg_drone/rotor_0_cmd -m gz.msgs.Double -p "data: 50.0" && '
             'gz topic -t /model/my_peg_drone/rotor_1_cmd -m gz.msgs.Double -p "data: 50.0" && '
             'gz topic -t /model/my_peg_drone/rotor_2_cmd -m gz.msgs.Double -p "data: -50.0" && '
             'gz topic -t /model/my_peg_drone/rotor_3_cmd -m gz.msgs.Double -p "data: -50.0"'],
        shell=True,
        output='screen'
    )

    return LaunchDescription([
        peg_x_arg,
        peg_y_arg,
        peg_z_arg,
        declare_world_arg,
        #gazebo_cmd,
        spawn_peg_node,
        # TimerAction(period=10.0, actions=[spin_propellers_cmd])
    ])
