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

# Funzione per estrarre parametri dall'airframe PX4
def get_airframe_params(model_name):
    """
    Cerca il file airframe corrispondente al modello e ne estrae i limiti di velocità dei motori.
    """
    import re
    airframes_dir = '/root/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes'
    w_min, w_max = 150.0, 1000.0  # Default fallback per x500
    
    try:
        if os.path.exists(airframes_dir):
            for filename in os.listdir(airframes_dir):
                path = os.path.join(airframes_dir, filename)
                with open(path, 'r') as f:
                    content = f.read()
                    # Cerchiamo il file che definisce questo modello
                    if f'PX4_SIM_MODEL:={model_name}' in content.replace(" ", ""):
                        # Estraiamo SIM_GZ_EC_MIN1 e MAX1 (assumiamo siano uguali per tutti i motori)
                        min_match = re.search(r'SIM_GZ_EC_MIN1\s+(\d+)', content)
                        max_match = re.search(r'SIM_GZ_EC_MAX1\s+(\d+)', content)
                        if min_match: w_min = float(min_match.group(1))
                        if max_match: w_max = float(max_match.group(1))
                        break
    except Exception as e:
        print(f"[AUTO-PHYSICS] Errore nel leggere i parametri airframe: {e}")
        
    return w_min, w_max

# Funzione per il calcolo automatico di massa, inerzia e posizione camera
def get_px4_model_info(model_name):
    """
    Scansiona ricorsivamente i file SDF di PX4 per ricavare massa, inerzia (Steiner), camera e spinta massima.
    """
    px4_path = '/root/PX4-Autopilot'
    models_dir = os.path.join(px4_path, 'Tools/simulation/gz/models')
    
    def parse_recursive(name, offset_pose=[0.0, 0.0, 0.0]):
        clean_name = name.replace("model://", "")
        sdf_path = os.path.join(models_dir, clean_name, 'model.sdf')
        
        if not os.path.exists(sdf_path):
            return 0.0, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.0, 0.0, 0.0, 0.0, 0.0, 0.016
        
        m_total = 0.0
        i_total = [0.0, 0.0, 0.0]
        cam_pos = [0.0, 0.0, 0.0]
        cam_rpy = [0.0, 0.0, 0.0]
        fov_h = 80.0
        fov_v = 60.0
        f_max_total = 0.0
        arm_l_x = 0.0
        arm_l_y = 0.0
        moment_constant = 0.016 # Default x500
        
        tree = ET.parse(sdf_path)
        root = tree.getroot()
        model_tag = root.find("model")
        if model_tag is None: return 0.0, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.0, 0.0, 0.0, 0.0, 0.0, 0.016

        # 1. Processa i Link locali
        for link in model_tag.findall("link"):
            l_pose = [0.0, 0.0, 0.0]
            p_tag = link.find("pose")
            if p_tag is not None:
                l_pose = [float(x) for x in p_tag.text.split()[:3]]
            
            # Posizione del link relativa all'origine del drone
            abs_link_x = l_pose[0] + offset_pose[0]
            abs_link_y = l_pose[1] + offset_pose[1]
            abs_link_z = l_pose[2] + offset_pose[2]

            inertial = link.find("inertial")
            if inertial is not None:
                # Recupero l'offset del baricentro interno al link
                i_pose = [0.0, 0.0, 0.0]
                ip_tag = inertial.find("pose")
                if ip_tag is not None:
                    i_pose = [float(x) for x in ip_tag.text.split()[:3]]

                m_tag = inertial.find("mass")
                if m_tag is not None:
                    m = float(m_tag.text)
                    m_total += m
                    
                    # Baricentro reale del link per Steiner
                    real_com_x = abs_link_x + i_pose[0]
                    real_com_y = abs_link_y + i_pose[1]
                    real_com_z = abs_link_z + i_pose[2]

                    inertia = inertial.find("inertia")
                    if inertia is not None:
                        i_total[0] += float(inertia.find("ixx").text) + m * (real_com_y**2 + real_com_z**2)
                        i_total[1] += float(inertia.find("iyy").text) + m * (real_com_x**2 + real_com_z**2)
                        i_total[2] += float(inertia.find("izz").text) + m * (real_com_x**2 + real_com_y**2)
            
            # 1.1 Estrazione bracci (per torque)
            if "rotor" in link.get("name", "").lower():
                arm_l_x = max(arm_l_x, abs(abs_link_x))
                arm_l_y = max(arm_l_y, abs(abs_link_y))
            
            # 1.2 Ricerca sensore camera (per cam_pos preciso)
            cam_sensor = link.find(".//sensor[@type='camera']")
            if cam_sensor is None:
                cam_sensor = link.find(".//sensor[@type='depth_camera']")

            if cam_sensor is not None:
                # Posa del sensore relativa al link
                s_pose = [0.0, 0.0, 0.0]
                sp_tag = cam_sensor.find("pose")
                if sp_tag is not None:
                    s_pose = [float(x) for x in sp_tag.text.split()[:3]]
                    sp_vals = [float(x) for x in sp_tag.text.split()]
                    if len(sp_vals) >= 6:
                        cam_rpy = sp_vals[3:6]
                
                # Posa finale della camera (Link + Sensore)
                cam_pos = [abs_link_x + s_pose[0], abs_link_y + s_pose[1], abs_link_z + s_pose[2]]
                
                # Estraiamo FOV
                cam_tag = cam_sensor.find("camera")
                if cam_tag is not None:
                    hfov_tag = cam_tag.find("horizontal_fov")
                    if hfov_tag is not None:
                        fov_h = float(hfov_tag.text) * 180.0 / 3.14159
                        fov_v = fov_h * (480.0/640.0) 
                        img_tag = cam_tag.find("image")
                        if img_tag is not None:
                            w = float(img_tag.find("width").text)
                            h = float(img_tag.find("height").text)
                            fov_v = fov_h * (h/w)

        # 2. Processa i motori
        for plugin in model_tag.findall(".//plugin[@name='gz::sim::systems::MulticopterMotorModel']"):
            k_tag = plugin.find("motorConstant")
            w_tag = plugin.find("maxRotVelocity")
            m_tag = plugin.find("momentConstant")
            if k_tag is not None and w_tag is not None:
                f_max_total += float(k_tag.text) * (float(w_tag.text)**2)
            if m_tag is not None:
                moment_constant = float(m_tag.text)

        # 3. Processa gli Include (ricorsivo)
        for include in model_tag.findall("include"):
            uri = include.find("uri")
            if uri is not None:
                inc_pose = [0.0, 0.0, 0.0]
                ip_tag = include.find("pose")
                if ip_tag is not None:
                    inc_pose = [float(x) for x in ip_tag.text.split()[:3]]
                
                new_offset = [a + b for a, b in zip(offset_pose, inc_pose)]
                m_inc, i_inc, c_inc, cr_inc, fh_inc, fv_inc, f_inc, lx_inc, ly_inc, mc_inc = parse_recursive(uri.text, new_offset)
                m_total += m_inc
                i_total = [a + b for a, b in zip(i_total, i_inc)]
                f_max_total += f_inc
                arm_l_x = max(arm_l_x, lx_inc)
                arm_l_y = max(arm_l_y, ly_inc)
                if mc_inc != 0.016: moment_constant = mc_inc
                if cam_pos == [0.0, 0.0, 0.0] and c_inc != [0.0, 0.0, 0.0]:
                    cam_pos = c_inc
                    cam_rpy = cr_inc
                    fov_h = fh_inc
                    fov_v = fv_inc
                
        return m_total, i_total, cam_pos, cam_rpy, fov_h, fov_v, f_max_total, arm_l_x, arm_l_y, moment_constant

    return parse_recursive(model_name)

def launch_setup(context, *args, **kwargs):
    # --- RECUPERO ARGOMENTI ---
    model_name = LaunchConfiguration('model').perform(context)
    planner_mode = LaunchConfiguration('planner_mode').perform(context)
    controller = LaunchConfiguration('controller').perform(context)
    MPC_controller = LaunchConfiguration('MPC_controller').perform(context)
    
    # --- CALCOLO AUTOMATICO FISICA ---
    auto_mass, auto_inertia, auto_cam, auto_cam_rpy, auto_fov_h, auto_fov_v, auto_fmax, auto_lx, auto_ly, auto_mc = get_px4_model_info(model_name)
    auto_wmin, auto_wmax = get_airframe_params(model_name)
    
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
            'cam_x': auto_cam[0], 'cam_y': auto_cam[1], 'cam_z': auto_cam[2] # AUTOMATICO
        }],
    )

    peg_planner = Node(
        package='drone_mpc_pkg', executable='peg_planner_node.py', name='peg_planner_node',
        parameters=[{
            'use_sim_time': True,
            'peg_start_x': peg_x, 'peg_start_y': peg_y, 'peg_start_z': peg_z,
            'px4_ns': 'px4_peg',
        }],
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

    return [
        ros_gz_bridge,
        peg_planner,
        TimerAction(period=2.0, actions=[mpc_planner_node]),
        data_logger,
        human_goal_node,
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config_file], condition=IfCondition(LaunchConfiguration('enable_rviz'))),
        Node(package='joy', executable='joy_node', parameters=[{'autorepeat_rate': 50.0}], condition=IfCondition(LaunchConfiguration('enable_joy')))
    ]

def generate_launch_description():
    env_pkg_dir = get_package_share_directory('gz_env_pkg')
    
    declared_arguments = [
        DeclareLaunchArgument('model', default_value='x500'),
        DeclareLaunchArgument('planner_mode', default_value='1'),
        DeclareLaunchArgument('MPC_controller', default_value='1'),
        DeclareLaunchArgument('controller', default_value='2'),
        DeclareLaunchArgument('enable_rviz', default_value='true'),
        DeclareLaunchArgument('enable_joy', default_value='true'),
        DeclareLaunchArgument('drone_x', default_value='0.0'),
        DeclareLaunchArgument('drone_y', default_value='0.0'),
        DeclareLaunchArgument('drone_z', default_value='0.0'),
        DeclareLaunchArgument('drone_yaw', default_value='1.5708'),
        DeclareLaunchArgument('peg_x', default_value='2.0'),
        DeclareLaunchArgument('peg_y', default_value='0.0'),
        DeclareLaunchArgument('peg_z', default_value='0.2'),
        DeclareLaunchArgument('cf', default_value='8.0e-4'),
        DeclareLaunchArgument('ct', default_value='1.0e-5'),
        DeclareLaunchArgument('world', default_value='default.sdf', description='Nome del mondo SDF da caricare'),
    ]

    peg_x = LaunchConfiguration('peg_x')
    peg_y = LaunchConfiguration('peg_y')
    peg_z = LaunchConfiguration('peg_z')

    spawn_environment = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(env_pkg_dir, 'launch', 'px4_gz.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'peg_x': peg_x,
            'peg_y': peg_y,
            'peg_z': peg_z
        }.items()
    )

    return LaunchDescription(declared_arguments + [
        spawn_environment,
        OpaqueFunction(function=launch_setup)
    ])
