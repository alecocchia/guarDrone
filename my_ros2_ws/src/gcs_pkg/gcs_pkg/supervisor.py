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

        # Subscriber per PoV effettivo cilindrico calcolato dall'MPC (/actual_pov: [r, beta, z_rel, yaw_rel])
        self.actual_pov = None
        self.create_subscription(Float64MultiArray, '/actual_pov', self.actual_pov_cb, 10)

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
        #self.pov_r = -1.0
        #self.pov_beta = 0.0

        self.init_pov_r = 3.0
        self.init_pov_beta = np.pi/2
        self.init_pov_z = 0.0
        self.init_pov_yaw = 0.0

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

        self.drone_target_sent = False

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
        # Posizione di preparazione a 2.0 m dalla parete (stessa x e z, offset su y)
        self.peg_mission_start_pose = np.array([self.contact_point[0], self.contact_point[1] + 2.0, self.contact_point[2]]) # ENU
        self.peg_mission_start_yaw = math.atan2(self.contact_point[1] - self.peg_mission_start_pose[1], self.contact_point[0] - self.peg_mission_start_pose[0]) # ENU
        self.has_inspected = False

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
    def actual_pov_cb(self, msg: Float64MultiArray):
        if len(msg.data) >= 3:
            self.actual_pov = [float(msg.data[0]), float(msg.data[1]), float(msg.data[2])]

    def transition_to_state(self, new_state):
        """Esegue una transizione di stato della FSM reimpostando opportunamente i flag."""
        self.get_logger().info(f"[FSM] Transizione verso lo stato: {new_state}")
        self.state = new_state
        self.drone_target_sent = False
        self.peg_target_sent = False
        self._operator_confirmed = False
        self.msg_cnt = 0

        # Se si entra in uno stato con MPC attivo, assicura che l'MPC sia acceso e il traj planner camera disattivato
        if new_state in ('HOME', 'MISSION_START', 'INSPECTION_START', 'INSPECTION_END'):
            if not self.task_started:
                msg_traj = Bool()
                msg_traj.data = False
                self.cam_traj_enabled_pub.publish(msg_traj)

                msg_start = Bool()
                msg_start.data = True
                self.task_start_pub.publish(msg_start)
                self.task_started = True

        if new_state == 'INSPECTION_END':
            self.has_inspected = True
        elif new_state == 'LANDING':
            # Notifica prima la transizione allo stato LANDING
            self._publish_state_if_changed()

            msg_stop = Bool()
            msg_stop.data = False
            self.task_start_pub.publish(msg_stop)
            self.peg_traj_enabled_pub.publish(msg_stop)
            self.cam_traj_enabled_pub.publish(msg_stop)

            self.publish_command(self.cmd_pub_1, 1, VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.publish_command(self.cmd_pub_2, 2, VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.get_logger().info("Comando di Atterraggio (VEHICLE_CMD_NAV_LAND) inviato. Attesa disarmo...")
            self.state = 'DISARM_WAIT'

        self._publish_state_if_changed()

    def _keyboard_input_cb(self, msg):
        """Callback per comandi da tastiera dell'operatore (sequenziali o randomici)."""
        cmd = msg.data.strip().lower()
        if cmd == 'ok':
            self._operator_confirmed = True
            self.get_logger().info('Conferma operatore ricevuta ("ok").')
        elif cmd == 'takeoff':
            self._operator_confirmed = True
            if self.state in ('WAIT_EKF', 'WAIT_START'):
                self.get_logger().info('Comando "takeoff" ricevuto! Avvio decollo (non appena EKF e offboard sono pronti).')
            else:
                self.get_logger().warn(f'Comando "takeoff" ricevuto nello stato {self.state}.')
        elif cmd in ('home', 'return_home',
                     'mission_start', 'mission_preparation',
                     'inspection_start', 'mission',
                     'inspection_end', 'detachment',
                     'landing', 'land'):
            alias_map = {
                'home': 'HOME',
                'return_home': 'HOME',
                'mission_start': 'MISSION_START',
                'mission_preparation': 'MISSION_START',
                'inspection_start': 'INSPECTION_START',
                'mission': 'INSPECTION_START',
                'inspection_end': 'INSPECTION_END',
                'detachment': 'INSPECTION_END',
                'landing': 'LANDING',
                'land': 'LANDING',
            }
            target_st = alias_map[cmd]
            self.get_logger().warn(f'COMANDO DIRETTO FASE RICEVUTO: "{cmd}" -> Salto immediato a {target_st}')
            self.transition_to_state(target_st)
        elif cmd == 'stop':
            self.get_logger().error('COMANDO STOP RICEVUTO! Atterraggio d\'emergenza!')
            msg_stop = Bool()
            msg_stop.data = False
            self.task_start_pub.publish(msg_stop)
            self.peg_traj_enabled_pub.publish(msg_stop)
            self.cam_traj_enabled_pub.publish(msg_stop)
            self.publish_command(self.cmd_pub_1, 1, VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.publish_command(self.cmd_pub_2, 2, VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.state = 'EMERGENCY'
        else:
            self.get_logger().warn(f'Comando non riconosciuto: "{cmd}"')

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
        """Restituisce il target PoV (r, beta, z) letto dal topic /actual_pov dell'MPC,
        oppure i valori iniziali fissi se /actual_pov non è ancora disponibile."""
        if self.actual_pov is not None:
            return self.actual_pov[0], self.actual_pov[1], self.actual_pov[2]
        return self.init_pov_r, self.init_pov_beta, self.init_pov_z

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
                    self.get_logger().info('EKF convergenti. In attesa comando ("takeoff" o "ok") per avviare il decollo...')
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
                self.get_logger().info("Droni in quota + conferma operatore! Passo a HOME.")
                self.transition_to_state('HOME')
            elif d1_up and d2_up and not self._operator_confirmed:
                if self.msg_cnt == 0:
                    self.get_logger().info('Droni in quota. In attesa conferma operatore ("ok" o "home") per HOME...')
                    # Pubblica target PoV iniziale fisso già durante l'hovering in quota per allineare logger e MPC
                    self.publish_pov_target(self.init_pov_r, self.init_pov_beta, self.init_pov_z, self.init_pov_yaw)
                    self.msg_cnt += 1

        elif self.state == 'HOME':
            if self._check_emergency():
                return

            home_target = np.array([self.peg_start_x, self.peg_start_y, self.takeoff_alt_2])

            # DRONE 1 (CAMERA): PoV iniziale fisso (centrato)
            if not self.drone_target_sent:
                r, beta, z = self.compute_actual_pov_target()
                self.publish_pov_target(r, beta, z, self.init_pov_yaw)
                self.drone_target_sent = True

            # DRONE 2 (INTERACTION): Posa target HOME
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
                self.get_logger().info(f"Target HOME inviato al Drone 2: {home_target}. Drone 1 segue in MPC (PoV centrato).")

            d2_local_ENU = self._M_NED2ENU @ np.array([self.drone2_local_pos.x, self.drone2_local_pos.y, self.drone2_local_pos.z])
            d2_pos = np.array([d2_local_ENU[0] + self.peg_start_x,
                               d2_local_ENU[1] + self.peg_start_y,
                               d2_local_ENU[2] + self.peg_start_z])
            d2_dist = np.linalg.norm(d2_pos - home_target)

            next_st = 'LANDING' if self.has_inspected else 'MISSION_START'

            if d2_dist < 0.25 and self._operator_confirmed:
                self.get_logger().info(f"Drone 2 in posizione HOME + conferma operatore! Passo a {next_st}.")
                self.transition_to_state(next_st)
            elif d2_dist < 0.25 and not self._operator_confirmed:
                if self.msg_cnt == 0:
                    self.get_logger().info(f'Drone 2 in posizione HOME. In attesa conferma operatore ("ok" per {next_st} o nome fase)...')
                    self.msg_cnt += 1

        elif self.state == 'MISSION_START':
            if self._check_emergency():
                return

            # DRONE 1 (CAMERA): PoV centrato
            if not self.drone_target_sent:
                r_target, beta_target, z_target = self.compute_actual_pov_target()
                h_frac = 0.0
                v_frac = 0.0
                yaw_off, z_off = self.compute_decentered_pov(r_target, h_frac, v_frac)
                self.publish_pov_target(r_target, beta_target, z_off, yaw_off)
                self.drone_target_sent = True

            # DRONE 2 (INTERACTION): Target missione a 2 m dalla parete
            yaw_target = self.peg_mission_start_yaw
            peg_target = self.peg_mission_start_pose

            if not self.peg_target_sent:
                peg_pose_target = PoseStamped()
                peg_pose_target.header.frame_id = 'world'
                peg_pose_target.pose.position.x = float(peg_target[0])
                peg_pose_target.pose.position.y = float(peg_target[1])
                peg_pose_target.pose.position.z = float(peg_target[2])
                rot = Rotation.from_euler('xyz', [0.0, 0.0, yaw_target])
                quat = rot.as_quat()
                peg_pose_target.pose.orientation.x = float(quat[0])
                peg_pose_target.pose.orientation.y = float(quat[1])
                peg_pose_target.pose.orientation.z = float(quat[2])
                peg_pose_target.pose.orientation.w = float(quat[3])
                self.peg_target_pub.publish(peg_pose_target)
                self.peg_target_sent = True
                self.get_logger().info(f"Target MISSION_START (2m dal muro) inviato a Drone 2: {peg_target}")

            d2_local_ENU = self._M_NED2ENU @ np.array([self.drone2_local_pos.x, self.drone2_local_pos.y, self.drone2_local_pos.z])
            d2_pos = np.array([d2_local_ENU[0] + self.peg_start_x,
                               d2_local_ENU[1] + self.peg_start_y,
                               d2_local_ENU[2] + self.peg_start_z])
            d2_dist = np.linalg.norm(d2_pos - peg_target)
            d2_yaw_ned = self.drone2_local_pos.heading
            d2_yaw_enu = (math.pi / 2.0) - d2_yaw_ned
            yaw_is_aligned = abs(min_angle(d2_yaw_enu - yaw_target)) < 0.15

            if d2_dist < 0.2 and yaw_is_aligned and self._operator_confirmed:
                self.get_logger().info("Drone interaction a 2m dal muro + allineato + conferma operatore. Passo a INSPECTION_START.")
                self.transition_to_state('INSPECTION_START')
            elif d2_dist < 0.2 and yaw_is_aligned and not self._operator_confirmed:
                if self.msg_cnt == 0:
                    self.get_logger().info('Drone 2 a 2m dal muro. In attesa conferma operatore ("ok" per INSPECTION_START o nome fase)...')
                    self.msg_cnt += 1

        elif self.state == 'INSPECTION_START':
            if self._check_emergency():
                return

            # DRONE 1 (CAMERA): PoV decentrato (drone osservato in basso a destra)
            if not self.drone_target_sent:
                r_target, beta_target, z_target = self.compute_actual_pov_target()
                h_frac = 0.4
                v_frac = 0.4
                yaw_off, z_off = self.compute_decentered_pov(r_target, h_frac, v_frac)
                self.publish_pov_target(r_target, beta_target, z_off, yaw_off)
                self.drone_target_sent = True

            # DRONE 2 (INTERACTION): Target contatto a parete
            if not self.peg_target_sent and self.task_started:
                msg_peg_target = PoseStamped()
                msg_peg_target.header.frame_id = 'world'
                msg_peg_target.pose.position.x = float(self.contact_point[0])
                msg_peg_target.pose.position.y = float(self.contact_point[1])
                msg_peg_target.pose.position.z = float(self.contact_point[2])
                target_yaw = self.peg_mission_start_yaw
                rot = Rotation.from_euler('xyz', [0.0, 0.0, target_yaw])
                quat = rot.as_quat()
                msg_peg_target.pose.orientation.x = float(quat[0])
                msg_peg_target.pose.orientation.y = float(quat[1])
                msg_peg_target.pose.orientation.z = float(quat[2])
                msg_peg_target.pose.orientation.w = float(quat[3])
                self.peg_target_pub.publish(msg_peg_target)
                self.peg_target_sent = True
                self.get_logger().info("Target di contatto a parete inviato a Drone 2. Inizio fase di ispezione a parete.")

            if self.peg_target_sent:
                if self._operator_confirmed:
                    self.get_logger().info("Conferma operatore ricevuta: Fine ispezione a parete. Passo a INSPECTION_END.")
                    self.transition_to_state('INSPECTION_END')
                elif self.msg_cnt == 0:
                    self.get_logger().info('In ispezione a parete. Digita "ok" (o "inspection_end") per staccarsi dalla parete.')
                    self.msg_cnt += 1

        elif self.state == 'INSPECTION_END':
            if self._check_emergency():
                return

            # DRONE 1 (CAMERA): PoV ritorna centrato
            if not self.drone_target_sent:
                r_target, beta_target, z_target = self.compute_actual_pov_target()
                h_frac = 0.0
                v_frac = 0.0
                yaw_off, z_off = self.compute_decentered_pov(r_target, h_frac, v_frac)
                self.publish_pov_target(r_target, beta_target, z_off, yaw_off)
                self.drone_target_sent = True

            d2_local_ENU = self._M_NED2ENU @ np.array([self.drone2_local_pos.x, self.drone2_local_pos.y, self.drone2_local_pos.z])
            d2_pos = np.array([d2_local_ENU[0] + self.peg_start_x,
                               d2_local_ENU[1] + self.peg_start_y,
                               d2_local_ENU[2] + self.peg_start_z])

            if not self.peg_target_sent:
                detach_distance = 1.0  # [m] arretramento di 1 metro come richiesto
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
                self.get_logger().info(f"Target di distacco (1m indietro) inviato a Drone 2: {self.detach_target}. Drone 1 torna a PoV centrato.")

            d2_dist = np.linalg.norm(d2_pos - self.detach_target)

            if d2_dist < 0.15 and self._operator_confirmed:
                self.get_logger().info("Distacco di 1m completato + conferma operatore! Passo a HOME.")
                self.transition_to_state('HOME')
            elif d2_dist < 0.2 and not self._operator_confirmed:
                if self.msg_cnt == 0:
                    self.get_logger().info('Drone 2 staccato dalla parete di 1m. In attesa "ok" (o "home") per HOME...')
                    self.msg_cnt += 1

        elif self.state == 'LANDING':
            # Stop MPC Task on Drone 1 e traj planner su Drone 2
            msg_stop = Bool()
            msg_stop.data = False
            self.task_start_pub.publish(msg_stop)
            self.peg_traj_enabled_pub.publish(msg_stop)
            self.cam_traj_enabled_pub.publish(msg_stop)

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