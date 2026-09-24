#!/usr/bin/env python3
# Il supervisore deve: 
# 1. Attendere che i droni siano pronti per il takeoff, armarli e farli decollare ad un'altezza prestabilita (es. 2 m)
# 2. Quando i droni sono in quota, passare in offboard
# 3. Iniziare il task di inseguimento
# 4. Fare landing quando hanno finito
# 5. Gestire emergenze
# Inoltre sarà lui a gestire tutte le richieste di offboard ed i check di sicurezza

import rclpy
import numpy as np
import math
from scipy.spatial.transform import Rotation
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleControlMode, VehicleCommand, VehicleLocalPosition
from std_msgs.msg import Bool, String, Float64MultiArray
from geometry_msgs.msg import PoseStamped
from utils_pkg.utils_np import min_angle

class SupervisorNode(Node):
    def __init__(self):
        super().__init__('supervisor_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        qos_latched = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            depth=1
        )

        # MATRICI DI CONVERSIONE FISSE
        self._M_NED2ENU = np.array([[0., 1., 0 ],
                        [1., 0., 0.],
                        [0., 0., -1.]])

        # Publishers for VehicleCommand (Drone 1 and Drone 2)
        self.cmd_pub_1 = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.cmd_pub_2 = self.create_publisher(VehicleCommand, '/px4_1/fmu/in/vehicle_command', qos_profile)

        # Publisher for Task Start
        self.task_start_pub = self.create_publisher(Bool, '/mpc_task/start', qos_latched)
        # Publisher per segnale di avvio logging (al momento dell'arming+offboard)
        self.logging_start_pub = self.create_publisher(Bool, '/logging/start', qos_latched)
        # Publisher per stato FSM del supervisore
        self.state_pub = self.create_publisher(String, '/supervisor/state', qos_latched)
        self._prev_published_state = None

        # Publisher per i nodi trajectory_planner
        self.cam_target_pub = self.create_publisher(PoseStamped, '/camera_target_pose', 10)
        self.peg_target_pub = self.create_publisher(PoseStamped, '/peg_target_pose', 10)
        self.cam_traj_enabled_pub = self.create_publisher(Bool, '/camera_traj_enabled', 10)
        self.peg_traj_enabled_pub = self.create_publisher(Bool, '/peg_traj_enabled', 10)

        # Publisher per target PoV cilindrico per MPC [r, beta, z_rel, yaw_offset]
        self.pov_pub = self.create_publisher(Float64MultiArray, '/pov_target', 10)

        # State Variables
        self.drone1_local_pos = VehicleLocalPosition()
        self.drone2_local_pos = VehicleLocalPosition()
        self.drone1_mode = VehicleControlMode()
        self.drone2_mode = VehicleControlMode()

        # Subscribers Drone 1
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.pos1_cb, qos_profile)
        self.create_subscription(VehicleControlMode, '/fmu/out/vehicle_control_mode', self.mode1_cb, qos_profile)

        # Subscribers Drone 2
        self.create_subscription(VehicleLocalPosition, '/px4_1/fmu/out/vehicle_local_position', self.pos2_cb, qos_profile)
        self.create_subscription(VehicleControlMode, '/px4_1/fmu/out/vehicle_control_mode', self.mode2_cb, qos_profile)

        # Parameters
        self.declare_parameter('takeoff_alt_1', 4.52+3.0) # Camera takeoff in ENU
        self.declare_parameter('takeoff_alt_2', 4.52+3.0) # Peg takeoff in ENU
        self.declare_parameter('cam_start_x', 0.0)
        self.declare_parameter('cam_start_y', 0.0)
        self.declare_parameter('cam_start_z', 4.52)
        self.declare_parameter('peg_start_x', 3.0)
        self.declare_parameter('peg_start_y', 0.0)
        self.declare_parameter('peg_start_z', 4.52)
        # Offset camera nel frame body (ENU)
        self.declare_parameter('cam_offset_x', 0.0)
        self.declare_parameter('cam_offset_y', 0.0)
        # Offset z camera nel frame body (ENU z up): cam_body_takeoff = takeoff_alt_1 - cam_z_offset
        # In questo modo la CAMERA si trova a takeoff_alt_1 dopo il decollo, non il centro del drone.
        self.declare_parameter('cam_z_offset', 0.0)

        # Parametri target PoV opzionali (se pov_r > 0 forza un valore fisso, altrimenti calcolato da hovering)
        self.pov_r = -1.0
        self.pov_beta = 0.0
        self.pov_z = 0.0
        self.pov_yaw = 0.0

        # Parametri FoV camera per calcolo offset decentrato
        self.declare_parameter('fov_h_deg', 80.0)      # FoV orizzontale camera [deg]
        self.declare_parameter('fov_v_deg', 60.0)      # FoV verticale camera [deg]

        self.takeoff_alt_1  = self.get_parameter('takeoff_alt_1').value
        self.takeoff_alt_2  = self.get_parameter('takeoff_alt_2').value
        self.cam_start_x    = self.get_parameter('cam_start_x').value
        self.cam_start_y    = self.get_parameter('cam_start_y').value
        self.cam_start_z    = self.get_parameter('cam_start_z').value
        self.peg_start_x    = self.get_parameter('peg_start_x').value
        self.peg_start_y    = self.get_parameter('peg_start_y').value
        self.peg_start_z    = self.get_parameter('peg_start_z').value
        self.cam_offset_x   = self.get_parameter('cam_offset_x').value
        self.cam_offset_y   = self.get_parameter('cam_offset_y').value
        self.cam_z_offset   = self.get_parameter('cam_z_offset').value

        self.fov_h_deg = self.get_parameter('fov_h_deg').value
        self.fov_v_deg = self.get_parameter('fov_v_deg').value

        self.state = 'WAIT_EKF'
        self.task_started = False
        self.peg_target_sent = False
        self.mission_start_received = False
        self.task_goal_pose_received = False
        self.mpc_ready = False

        self.target_to_send = False

        # Flag conferma operatore (da tastiera)
        self._operator_confirmed = False
        self.msg_cnt = 0
        
        # Subscriber for manual mission start
        self.create_subscription(Bool, '/mission/start', self.mission_start_cb, qos_latched)
        self.create_subscription(Bool, '/drone_planner_ready', self.mpc_ready_cb, qos_latched)
        # Subscriber per comandi da tastiera dell'operatore
        self.create_subscription(String, '/keyboard_input', self._keyboard_input_cb, 10)

        # Initial and final poses for interaction drone
        self.contact_point = np.array([self.peg_start_x, -55.95, 9.5])  # ENU
        # Let the mission begin in front of the wall (same x and z, offset on y)
        self.peg_mission_start_pose = np.array([self.contact_point[0], self.contact_point[1] + 3.0, self.contact_point[2]]) # ENU
        self.peg_mission_start_yaw = math.atan2(self.contact_point[1] - self.peg_mission_start_pose[1], self.contact_point[0] - self.peg_mission_start_pose[0]) # ENU

        self.get_logger().info("Supervisor Node Avviato. Stato: WAIT_EKF")
    
        # Main loop at 10 Hz
        self.timer = self.create_timer(0.1, self.loop)

    def mission_start_cb(self, msg):
        if msg.data and not self.mission_start_received:
            self.mission_start_received = True
            self.get_logger().info("Ricevuto segnale MANUALE su /mission/start!")

    def mpc_ready_cb(self, msg):
        if msg.data and not self.mpc_ready:
            self.mpc_ready = True
            self.get_logger().info("Ricevuto segnale MPC Pronto!")

    def pos1_cb(self, msg): self.drone1_local_pos = msg
    def pos2_cb(self, msg): self.drone2_local_pos = msg
    def mode1_cb(self, msg): self.drone1_mode = msg
    def mode2_cb(self, msg): self.drone2_mode = msg

    def _keyboard_input_cb(self, msg):
        """Callback per comandi da tastiera dell'operatore."""
        cmd = msg.data.strip().lower()
        if cmd == 'ok':
            self._operator_confirmed = True
            self.get_logger().info('Conferma operatore ricevuta ("ok").')
        elif cmd == 'land':
            self.get_logger().warn('COMANDO "land" RICEVUTO! Avvio immediato della sequenza di atterraggio.')
            self.state = 'LANDING'
            self._operator_confirmed = False
        elif cmd == 'stop':
            self.get_logger().error('COMANDO STOP RICEVUTO! Atterraggio d\'emergenza!')
            self.publish_command(self.cmd_pub_1, 1, VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.publish_command(self.cmd_pub_2, 2, VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.state = 'EMERGENCY'

    def publish_command(self, publisher, target_sys, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = target_sys
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        publisher.publish(msg)

    def compute_actual_pov_target(self):
        """Calcola il target PoV (r, beta, z) dai parametri o dalla posa reale in hovering."""
        if self.pov_r > 0.0:
            return self.pov_r, self.pov_beta, self.pov_z
        d1_local_ENU = self._M_NED2ENU @ np.array([self.drone1_local_pos.x, self.drone1_local_pos.y, self.drone1_local_pos.z])
        d1_pos = d1_local_ENU + np.array([self.cam_start_x, self.cam_start_y, self.cam_start_z])
        d2_local_ENU = self._M_NED2ENU @ np.array([self.drone2_local_pos.x, self.drone2_local_pos.y, self.drone2_local_pos.z])
        d2_pos = d2_local_ENU + np.array([self.peg_start_x, self.peg_start_y, self.peg_start_z])

        d1_yaw_enu = (math.pi / 2.0) - self.drone1_local_pos.heading
        R_body = Rotation.from_euler('xyz', [0.0, 0.0, d1_yaw_enu]).as_matrix()
        cam_pos = d1_pos + R_body @ np.array([self.cam_offset_x, self.cam_offset_y, self.cam_z_offset])

        dx = float(cam_pos[0] - d2_pos[0])
        dy = float(cam_pos[1] - d2_pos[1])
        dz = float(cam_pos[2] - d2_pos[2])

        r_target = math.sqrt(dx**2 + dy**2)
        beta_target = math.atan2(dy, dx)
        z_target = dz
        return r_target, beta_target, z_target

    def compute_decentered_pov(self, r, h_fraction=0.0, v_fraction=0.0):
        """Calcola yaw_offset e z_rel per decentrare l'oggetto nel FoV.

        h_fraction in [-1, 1]: +1 = bordo destro, -1 = bordo sinistro
        v_fraction in [-1, 1]: +1 = bordo superiore, -1 = bordo inferiore
        Restituisce (yaw_offset, z_rel) da passare a publish_pov_target.
        """
        fov_h_half = math.radians(self.get_parameter('fov_h_deg').value / 2.0)
        fov_v_half = math.radians(self.get_parameter('fov_v_deg').value / 2.0)

        # Offset angolare orizzontale: positivo = oggetto a destra nel frame
        yaw_offset = h_fraction * fov_h_half

        # Offset verticale: usa la geometria prospettica con lo yaw_offset corrente
        # z_max = r * cos(yaw_offset) * tan(fov_v/2)  (come nel nodo haptic)
        z_max = r * math.cos(yaw_offset) * math.tan(fov_v_half)
        z_rel = v_fraction * z_max

        return yaw_offset, z_rel

    def publish_pov_target(self, r, beta, z=0.0, yaw=0.0):
        """Pubblica riferimento PoV cilindrico [r, beta, z_rel, yaw_offset] per l'MPC."""
        msg = Float64MultiArray()
        msg.data = [float(r), float(beta), float(z), float(yaw)]
        self.pov_pub.publish(msg)
        self.get_logger().info(f"Target PoV inviato all'MPC: r={r:.3f} m, beta={math.degrees(beta):.1f}°, z={z:.3f} m, yaw={math.degrees(yaw):.1f}°")

    def _publish_state_if_changed(self):
        """Pubblica lo stato della FSM su /supervisor/state ad ogni transizione."""
        if self.state != self._prev_published_state:
            self._prev_published_state = self.state
            msg_state = String()
            msg_state.data = self.state
            self.state_pub.publish(msg_state)
            self.get_logger().info(f"[FSM] Transizione di stato pubblicata: {self.state}")

    def loop(self):
        self._publish_state_if_changed()
        if self.state == 'WAIT_EKF':
            # Check if both drones have full EKF convergence
            d1_ekf_ok = (self.drone1_local_pos.timestamp > 0 and 
                         self.drone1_local_pos.xy_valid and 
                         self.drone1_local_pos.z_valid)
            d2_ekf_ok = (self.drone2_local_pos.timestamp > 0 and 
                         self.drone2_local_pos.xy_valid and 
                         self.drone2_local_pos.z_valid)

            if d1_ekf_ok and d2_ekf_ok and self._operator_confirmed:
                self.get_logger().info("EKF Convergenti per entrambi + conferma operatore. Passo a WAIT_START")
                self.state = 'WAIT_START'
                self.mission_start_received = True
                self._operator_confirmed = False
                self.msg_cnt = 0
            elif d1_ekf_ok and d2_ekf_ok and not self._operator_confirmed:
                if self.msg_cnt == 0:
                    self.get_logger().info('EKF convergenti. In attesa conferma operatore ("ok") per procedere...')
                    self.msg_cnt += 1
            else:
                # Logghiamo cosa manca ogni 2 secondi
                if not hasattr(self, 'last_log_time'):
                    self.last_log_time = self.get_clock().now()
                elif (self.get_clock().now() - self.last_log_time).nanoseconds > 2e9:
                    self.get_logger().info(f"In attesa EKF... D1_EKF: {d1_ekf_ok}, D2_EKF: {d2_ekf_ok}")
                    self.last_log_time = self.get_clock().now()

        elif self.state == 'WAIT_START':
            if self.mission_start_received and self.mpc_ready:
                self.get_logger().info("Segnale di avvio e MPC Pronto ricevuti. Mando comandi di decollo e Passo a ARM_OFFBOARD")
                
                # Invia comando di decollo al camera drone.
                # Il body deve salire a (takeoff_alt_1 - cam_z_offset) in modo che la
                # CAMERA si trovi a takeoff_alt_1 (= quota target = quota del peg).
                cam_body_takeoff_z = float(self.takeoff_alt_1 - self.cam_z_offset)
                cam_pose = PoseStamped()
                cam_pose.header.frame_id = 'world'
                cam_pose.pose.position.x = float(self.cam_start_x)
                cam_pose.pose.position.y = float(self.cam_start_y)
                cam_pose.pose.position.z = cam_body_takeoff_z
                self.get_logger().info(f"Posizione di decollo al camera drone (body z): {cam_pose.pose.position.z:.3f} m")
                self.cam_target_pub.publish(cam_pose)

                # Invia comando di decollo al peg drone
                peg_pose = PoseStamped()
                peg_pose.header.frame_id = 'world'
                peg_pose.pose.position.x = float(self.peg_start_x)
                peg_pose.pose.position.y = float(self.peg_start_y)
                peg_pose.pose.position.z = float(self.takeoff_alt_2) # ENU
                self.get_logger().info(f"Posizione di decollo al peg drone: {peg_pose.pose.position.x}, {peg_pose.pose.position.y}, {peg_pose.pose.position.z}")
                self.peg_target_pub.publish(peg_pose)
                
                # Assicurati che i planner siano accesi
                msg = Bool()
                msg.data = True
                self.cam_traj_enabled_pub.publish(msg)
                self.peg_traj_enabled_pub.publish(msg)

                self.state = 'ARM_OFFBOARD'
                # Segnala al logger di iniziare a registrare
                log_start_msg = Bool()
                log_start_msg.data = True
                self.logging_start_pub.publish(log_start_msg)
                self.get_logger().info("Segnale /logging/start pubblicato.")

        elif self.state == 'ARM_OFFBOARD':
            self.get_logger().info(f"Stato droni -> D1(offboard:{self.drone1_mode.flag_control_offboard_enabled}, arm:{self.drone1_mode.flag_armed}) | D2(offboard:{self.drone2_mode.flag_control_offboard_enabled}, arm:{self.drone2_mode.flag_armed})")
            d1_ready = (self.drone1_mode.flag_control_offboard_enabled and self.drone1_mode.flag_armed)
            d2_ready = (self.drone2_mode.flag_control_offboard_enabled and self.drone2_mode.flag_armed)

            if not d1_ready:
                self.publish_command(self.cmd_pub_1, 1, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.publish_command(self.cmd_pub_1, 1, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            
            if not d2_ready:
                self.publish_command(self.cmd_pub_2, 2, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.publish_command(self.cmd_pub_2, 2, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

            if d1_ready and d2_ready:
                self.get_logger().info("Entrambi i droni sono Armati e in Offboard. Monitoraggio Decollo...")
                self.state = 'TAKEOFF_MONITOR'

        elif self.state == 'TAKEOFF_MONITOR':
            # Check altitudes (z is negative upwards)
            # drone_local_pos.z è NED (negativo verso l'alto), relativo allo spawn point.
            # La quota body target è (takeoff_alt_1 - cam_z_offset).
            d1_up = abs(-self.drone1_local_pos.z - (self.takeoff_alt_1 - self.cam_z_offset - self.cam_start_z)) < 0.1
            d2_up = abs(-self.drone2_local_pos.z - (self.takeoff_alt_2 - self.peg_start_z)) < 0.1
            self.get_logger().info(f"d1_up: {self.drone1_local_pos.z}")
            self.get_logger().info(f"d2_up: {self.drone2_local_pos.z}")

            if d1_up and d2_up and self._operator_confirmed:
                self.get_logger().info("Droni in quota + conferma operatore! Disabilito traj planner camera e invio segnale di START all'MPC.")
                
                # 1. Spegne offboard_trajectory_planner del camera drone
                msg_traj = Bool()
                msg_traj.data = False
                self.cam_traj_enabled_pub.publish(msg_traj)

                # 2. Calcola e pubblica target PoV iniziale per l'MPC
                r_target, beta_target, z_target = self.compute_actual_pov_target()
                h_frac = 0.0
                v_frac = 0.0
                yaw_off, z_off = self.compute_decentered_pov(r_target, h_frac, v_frac)
                self.publish_pov_target(r_target, beta_target, z_off, yaw_off)

                # 3. Accende MPC
                msg_start = Bool()
                msg_start.data = True
                self.task_start_pub.publish(msg_start)
                
                self.task_started = True
                self.state = 'MISSION_PREPARATION'
                self._operator_confirmed = False
                self.msg_cnt = 0
                self.target_to_send = True
            elif d1_up and d2_up and not self._operator_confirmed:
                if self.msg_cnt == 0:
                    self.get_logger().info('Droni in quota. In attesa conferma operatore ("ok") per MISSION_PREPARATION...')
                    # Pubblica target PoV già durante l'hovering in quota per allineare logger e MPC prima dello switch
                    r_target, beta_target, z_target = self.compute_actual_pov_target()
                    h_frac = 0.0
                    v_frac = 0.0
                    yaw_off, z_off = self.compute_decentered_pov(r_target, h_frac, v_frac)
                    self.publish_pov_target(r_target, beta_target, z_off, yaw_off)
                    self.msg_cnt += 1
        
        elif self.state == 'MISSION_PREPARATION':
            
            ## DRONE 1 (CAMERA)
            if self.target_to_send:
                r_target, beta_target, z_target = self.compute_actual_pov_target()
                h_frac = 0.0
                v_frac = 0.0
                yaw_off, z_off = self.compute_decentered_pov(r_target, h_frac, v_frac)
                self.publish_pov_target(r_target, beta_target, z_off, yaw_off)
                self.target_to_send = False
            ## DRONE 2 (INTERACTION)
            yaw_target = self.peg_mission_start_yaw # ENU

            # Check if d2 is at the target position for starting the mission
            peg_target_x =self.peg_mission_start_pose[0]        # ENU
            peg_target_y =self.peg_mission_start_pose[1]        # ENU
            peg_target_z = self.peg_mission_start_pose[2]       # ENU

            # Publish pose
            peg_pose_target = PoseStamped()
            peg_pose_target.header.frame_id = 'world'
            peg_pose_target.pose.position.x = float(peg_target_x)
            peg_pose_target.pose.position.y = float(peg_target_y)
            peg_pose_target.pose.position.z = float(peg_target_z)
            # Conversion from RPY to quaternion
            rot = Rotation.from_euler('xyz', [0.0, 0.0, yaw_target])
            quat = rot.as_quat()
            peg_pose_target.pose.orientation.x = float(quat[0])
            peg_pose_target.pose.orientation.y = float(quat[1])
            peg_pose_target.pose.orientation.z = float(quat[2])
            peg_pose_target.pose.orientation.w = float(quat[3])

            if not self.peg_target_sent:
                # Giving target to peg drone to prepare for the mission
                self.get_logger().info(f"Preparing for the mission and pose target for drone 2: {self.peg_mission_start_pose}")

                self.peg_target_pub.publish(peg_pose_target)
                self.peg_target_sent = True

            # CHECKS
            d2_local_ENU = self._M_NED2ENU @ np.array([self.drone2_local_pos.x, self.drone2_local_pos.y, self.drone2_local_pos.z])
            
            d2_x = d2_local_ENU[0] + self.peg_start_x # ENU
            d2_y = d2_local_ENU[1] + self.peg_start_y # ENU
            d2_z = d2_local_ENU[2] + self.peg_start_z # ENU

            d2_yaw_ned = self.drone2_local_pos.heading # NED
            d2_yaw_enu = (math.pi / 2.0) - d2_yaw_ned # ENU

            drone_interaction_in_position = (
                abs(d2_x - peg_target_x) < 0.1 and
                abs(d2_y - peg_target_y) < 0.1 and
                abs(d2_z - peg_target_z) < 0.1
            )
            yaw_is_aligned = abs(min_angle(d2_yaw_enu - yaw_target)) < 0.1

            
            if drone_interaction_in_position and yaw_is_aligned and self._operator_confirmed:
                self.get_logger().info("Drone interaction in posizione + conferma operatore. Passo a MISSION.")
                self.state = 'MISSION'
                self._operator_confirmed = False
                self.msg_cnt  = 0
                self.target_to_send = True
                self.peg_target_sent = False
            elif drone_interaction_in_position and yaw_is_aligned and not self._operator_confirmed:
                if self.msg_cnt == 0:
                    self.get_logger().info('Drone in posizione e allineato. In attesa conferma operatore ("ok") per MISSION...')
                    self.msg_cnt += 1
                
            else:
                self.get_logger().info(f"In attesa che il drone interaction raggiunga la posizione di missione ed allineamento. D2: ({d2_x}, {d2_y}, {d2_z}) -> ({peg_target_x}, {peg_target_y}, {peg_target_z})")

        elif self.state == 'MISSION':
            if self._check_emergency():
                return
            # DRONE 1
            if self.target_to_send :
                r_target, beta_target, z_target = self.compute_actual_pov_target()
                h_frac = 0.4
                v_frac = 0.4
                yaw_off, z_off = self.compute_decentered_pov(r_target, h_frac, v_frac)
                self.publish_pov_target(r_target, beta_target, z_off, yaw_off)
                self.target_to_send=False
                
            if not self.peg_target_sent and self.task_started:
                msg_peg_target = PoseStamped()
                msg_peg_target.header.frame_id = 'world'
                msg_peg_target.pose.position.x = float(self.contact_point[0])
                #msg_peg_target.pose.position.y = float(self.contact_point[1]+1.5)   #-55.91 for contact with wall;-14 for testing teleoperation with cube, -1.91
                msg_peg_target.pose.position.y = float(-55.95)
                msg_peg_target.pose.position.z = float(self.contact_point[2])
                
                # Impostazione del target di yaw desiderato (in radianti)
                target_yaw = self.peg_mission_start_yaw # ENU
                rot = Rotation.from_euler('xyz', [0.0, 0.0, target_yaw])
                quat = rot.as_quat()
                msg_peg_target.pose.orientation.x = float(quat[0])
                msg_peg_target.pose.orientation.y = float(quat[1])
                msg_peg_target.pose.orientation.z = float(quat[2])
                msg_peg_target.pose.orientation.w = float(quat[3])
                
                self.peg_target_pub.publish(msg_peg_target)
                self.peg_target_sent = True
                self.get_logger().info("Target di contatto inviato a Drone 2. Inizio fase di interazione a parete.")

            if self.peg_target_sent:
                if self._operator_confirmed:
                    self.get_logger().info("Conferma operatore ricevuta: Fine interazione a parete. Passo a DETACHMENT.")
                    self.state = 'DETACHMENT'
                    self._operator_confirmed = False
                    self.peg_target_sent = False
                    self.msg_cnt = 0
                elif self.msg_cnt == 0:
                    self.get_logger().info('In missione a parete. Digita "ok" per procedere al distacco (DETACHMENT) o "land" per atterrare.')
                    self.msg_cnt += 1

        elif self.state == 'DETACHMENT':
            if self._check_emergency():
                return

            # Posizione corrente reale di Drone 2 in ENU
            d2_local_ENU = self._M_NED2ENU @ np.array([self.drone2_local_pos.x, self.drone2_local_pos.y, self.drone2_local_pos.z])
            d2_pos = np.array([d2_local_ENU[0] + self.peg_start_x,
                               d2_local_ENU[1] + self.peg_start_y,
                               d2_local_ENU[2] + self.peg_start_z])

            if not self.peg_target_sent:
                # Calcola il target arretrando di 1.5m nella direzione opposta alla parete (asse z negativo del contatto)
                detach_distance = 0.5  # [m] arretramento dal punto di contatto
                detach_direction = -np.array([math.cos(self.peg_mission_start_yaw), math.sin(self.peg_mission_start_yaw), 0.0])
                self.detach_target = d2_pos + detach_distance * detach_direction

                peg_pose_target = PoseStamped()
                peg_pose_target.header.frame_id = 'world'
                peg_pose_target.pose.position.x = float(self.detach_target[0])
                peg_pose_target.pose.position.y = float(self.detach_target[1])
                peg_pose_target.pose.position.z = float(self.detach_target[2])
                rot = Rotation.from_euler('xyz', [0.0, 0.0, self.peg_mission_start_yaw])
                quat = rot.as_quat()
                peg_pose_target.pose.orientation.x = float(quat[0])
                peg_pose_target.pose.orientation.y = float(quat[1])
                peg_pose_target.pose.orientation.z = float(quat[2])
                peg_pose_target.pose.orientation.w = float(quat[3])
                self.peg_target_pub.publish(peg_pose_target)
                self.peg_target_sent = True
                self.get_logger().info(f"Target trapezoidale di distacco inviato al Drone 2: {self.detach_target}. Drone 1 continua a seguire in MPC.")

            # Verifica se Drone 2 ha raggiunto la posizione di distacco
            d2_dist = np.linalg.norm(d2_pos - self.detach_target)

            if d2_dist < 0.1 and self._operator_confirmed:
                self.get_logger().info("Distacco completato + conferma operatore! Passo a RETURN_HOME (posa desiderata iniziale di hovering).")
                self.state = 'RETURN_HOME'
                self.peg_target_sent = False
                self._operator_confirmed = False
                self.msg_cnt = 0
            elif d2_dist < 0.2 and not self._operator_confirmed:
                if self.msg_cnt == 0:
                    self.get_logger().info('Drone 2 staccato dalla parete in sicurezza. In attesa "ok" per RETURN_HOME (posa hovering iniziale)...')
                    self.msg_cnt += 1

        elif self.state == 'RETURN_HOME':
            if self._check_emergency():
                return

            home_target = np.array([self.peg_start_x, self.peg_start_y, self.takeoff_alt_2])
            if not self.peg_target_sent:
                peg_pose_target = PoseStamped()
                peg_pose_target.header.frame_id = 'world'
                peg_pose_target.pose.position.x = float(home_target[0])
                peg_pose_target.pose.position.y = float(home_target[1])
                peg_pose_target.pose.position.z = float(home_target[2])
                rot = Rotation.from_euler('xyz', [0.0, 0.0, self.peg_mission_start_yaw])
                quat = rot.as_quat()
                peg_pose_target.pose.orientation.x = float(quat[0])
                peg_pose_target.pose.orientation.y = float(quat[1])
                peg_pose_target.pose.orientation.z = float(quat[2])
                peg_pose_target.pose.orientation.w = float(quat[3])
                self.peg_target_pub.publish(peg_pose_target)
                self.peg_target_sent = True
                self.get_logger().info(f"Target di RITORNO inviato a Drone 2: {home_target}. Drone 1 continua a seguire in MPC.")

            d2_local_ENU = self._M_NED2ENU @ np.array([self.drone2_local_pos.x, self.drone2_local_pos.y, self.drone2_local_pos.z])
            d2_pos = np.array([d2_local_ENU[0] + self.peg_start_x,
                               d2_local_ENU[1] + self.peg_start_y,
                               d2_local_ENU[2] + self.peg_start_z])
            d2_dist = np.linalg.norm(d2_pos - home_target)

            if d2_dist < 0.25 and self._operator_confirmed:
                self.get_logger().info("Droni sopra lo spawn di decollo + conferma operatore! Passo a LANDING.")
                self.state = 'LANDING'
                self._operator_confirmed = False
                self.msg_cnt = 0
            elif d2_dist < 0.25 and not self._operator_confirmed:
                if self.msg_cnt == 0:
                    self.get_logger().info('Drone 2 sopra la base di decollo. In attesa "ok" per LANDING...')
                    self.msg_cnt += 1

        elif self.state == 'LANDING':
            # Stop MPC Task on Drone 1
            msg_stop = Bool()
            msg_stop.data = False
            self.task_start_pub.publish(msg_stop)

            # Comando NAV_LAND PX4 a entrambi i droni
            self.publish_command(self.cmd_pub_1, 1, VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.publish_command(self.cmd_pub_2, 2, VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.get_logger().info("Comando di Atterraggio (VEHICLE_CMD_NAV_LAND) inviato a entrambi i droni. Attesa disarmo...")
            self.state = 'DISARM_WAIT'

        elif self.state == 'DISARM_WAIT':
            d1_disarmed = not self.drone1_mode.flag_armed
            d2_disarmed = not self.drone2_mode.flag_armed
            if d1_disarmed and d2_disarmed:
                self.get_logger().info("Entrambi i droni sono atterrati e DISARMATI. Missione completata con successo!")
                self.state = 'MISSION_COMPLETE'

        elif self.state == 'MISSION_COMPLETE':
            pass

        elif self.state == 'EMERGENCY':
            # Keep sending land just in case
            self.get_logger().error("EMERGENZA: Atterraggio forzato per entrambi i droni")
            self.publish_command(self.cmd_pub_1, 1, VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.publish_command(self.cmd_pub_2, 2, VehicleCommand.VEHICLE_CMD_NAV_LAND)

        # Pubblica immediatamente ogni transizione avvenuta all'interno di loop()
        self._publish_state_if_changed()

    def _check_emergency(self):
        """Controlla se uno dei droni ha perso l'offboard o si e disarmato improvvisamente."""
        d1_fail = not self.drone1_mode.flag_armed or not self.drone1_mode.flag_control_offboard_enabled
        d2_fail = not self.drone2_mode.flag_armed or not self.drone2_mode.flag_control_offboard_enabled

        if d1_fail and self.drone2_mode.flag_armed:
            self.get_logger().error("EMERGENZA: Drone 1 ha fallito. Atterraggio Drone 2!")
            self.publish_command(self.cmd_pub_2, 2, VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.state = 'EMERGENCY'
            return True
        elif d2_fail and self.drone1_mode.flag_armed:
            self.get_logger().error("EMERGENZA: Drone 2 ha fallito. Atterraggio Drone 1!")
            self.publish_command(self.cmd_pub_1, 1, VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.state = 'EMERGENCY'
            return True
        return False

def main(args=None):
    rclpy.init(args=args)
    node = SupervisorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()