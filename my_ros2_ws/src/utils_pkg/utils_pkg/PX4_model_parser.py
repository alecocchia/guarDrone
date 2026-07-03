import os
import re
import xml.etree.ElementTree as ET

class PX4ModelParser:
    def __init__(self, px4_path='/root/PX4-Autopilot'):
        self.px4_path = px4_path
        self.models_dir = os.path.join(self.px4_path, 'Tools/simulation/gz/models')
        self.airframes_dir = os.path.join(self.px4_path, 'ROMFS/px4fmu_common/init.d-posix/airframes')

    def get_airframe_params(self, model_name):
        """
        Cerca il file airframe corrispondente al modello e ne estrae i limiti di velocità dei motori.
        """
        w_min, w_max = 150.0, 1000.0  # Default fallback per x500
        
        try:
            if os.path.exists(self.airframes_dir):
                for filename in os.listdir(self.airframes_dir):
                    path = os.path.join(self.airframes_dir, filename)
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

    def get_px4_model_info(self, model_name):
        """
        Scansiona ricorsivamente i file SDF di PX4 per ricavare massa, inerzia (Steiner), camera e spinta massima.
        """
        def parse_recursive(name, offset_pose=None):
            if offset_pose is None:
                offset_pose = [0.0, 0.0, 0.0]
                
            clean_name = name.replace("model://", "")
            sdf_path = os.path.join(self.models_dir, clean_name, 'model.sdf')
            
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
