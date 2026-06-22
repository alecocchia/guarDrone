#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

using std::placeholders::_1;

class FDHapticJoyNode : public rclcpp::Node
{
public:
  FDHapticJoyNode()
  : Node("fd_haptic_joy_node")
  {
    // Parametri molla aptica
    this->declare_parameter("k_spring", 50.0);    // Aumentato per un ritorno più forte (da 40)
    this->declare_parameter("b_damping", 10.0);    // Coefficiente di smorzamento viscoso virtuale
    this->declare_parameter("max_force", 15.0);    // Limite hardware
    this->declare_parameter("deadband", 0.005);    
    this->declare_parameter("joy_scale", 15.0);    
    
    // Parametri integrazione PoV 
    this->declare_parameter("v_pan_max", 0.7);   
    this->declare_parameter("v_zc_max", 0.5);  
    this->declare_parameter("v_xc_max", 1.2);
    this->declare_parameter("dt", 0.01);           

    // Parametri Campo Potenziale (Force Feedback dai vincoli MPC)
    this->declare_parameter("fov_h", 80.0);         
    this->declare_parameter("fov_v", 60.0);         
    this->declare_parameter("x_min_safety", 1.0);   
    this->declare_parameter("k_repulsive", 3.0); // costante elastica del campo repulsivo
    this->declare_parameter("alpha", 1.3); // esponente del campo repulsivo che definisce la forma dell'esponenziale
    this->declare_parameter("activation_ratio", 0.8); // Inizia a sentirsi prima (act_ratio%)
    this->declare_parameter("activation_ratio_cam", 0.5); // Inizia a sentirsi prima (act_ratio%)
    this->declare_parameter("max_repulsive_force", 15.0); // Aumentato limite repulsione

    // Actual parameters taken from launchfile:
    //  'k_spring'
    //  'b_damping'
    //  'v_pan_max'
    //  'v_zc_max'
    //  'v_xc_max'
    //  'deadband'

    // Subscribers
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/fd/ee_pose", 10, std::bind(&FDHapticJoyNode::pose_cb, this, _1));
    
    // Subscriber: bottone singolo /fd/button_state (Bool, backward compat — bottone centrale)
    button_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/fd/button_state", 10, std::bind(&FDHapticJoyNode::button_cb, this, _1));

    // Subscriber: tutti e 4 i bottoni /fd/button_states (Int32MultiArray)
    buttons_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
      "/fd/button_states", 10, std::bind(&FDHapticJoyNode::buttons_cb, this, _1));

    actual_pov_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/actual_pov", 10, std::bind(&FDHapticJoyNode::actual_pov_cb, this, _1));
    peg_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/peg_ref_pose", 10, std::bind(&FDHapticJoyNode::peg_pose_cb, this, _1));

    // Publishers
    force_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/fd/fd_controller/commands", 10);
    haptic_ref_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/haptic_ref", 10);
    peg_live_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/peg_live_pose", 10);

    // Initial state
    current_pov_ref_ = {2.0, 0.0, 0.0, 0.0}; // [Xc, Yc, Zc, Pan_mutuo]
    current_pov_vel_ = {0.0, 0.0, 0.0, 0.0}; // [dXc, dYc, dZc, dPan]
    actual_pov_ = {2.0, 0.0, 0.0, 0.0};       // [Xc, Yc, Zc, Pan] dal drone
    falcon_pos_ = {0.0, 0.0, 0.0};
    prev_falcon_pos_ = {0.0, 0.0, 0.0};
    falcon_vel_ = {0.0, 0.0, 0.0};
    first_pose_received_ = false;
    button_pressed_ = false;
    button_states_ = {0, 0, 0, 0};  // stato corrente dei 4 bottoni [btn0..btn3]
    peg_pos_enu_   = {0.0, 0.0, 0.0};
    peg_target_pos_= {0.0, 0.0, 0.0};
    peg_target_yaw_= 0.0;
    peg_yaw_enu_   = 0.0;
    peg_mode_active_ = false;

    // Timer loop a 100Hz
    double dt = this->get_parameter("dt").as_double();
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(dt), std::bind(&FDHapticJoyNode::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "Haptic Joy Node avviato con Force Feedback a Campo Potenziale.");
  }

private:
  void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    falcon_pos_[0] = msg->pose.position.x;
    falcon_pos_[1] = msg->pose.position.y;
    falcon_pos_[2] = msg->pose.position.z;
    if (!first_pose_received_) {
      prev_falcon_pos_[0] = falcon_pos_[0];
      prev_falcon_pos_[1] = falcon_pos_[1];
      prev_falcon_pos_[2] = falcon_pos_[2];
      first_pose_received_ = true;
    }
  }

  void button_cb(const std_msgs::msg::Bool::SharedPtr msg)
  {
    // Backward compat: bottone 0 (quello centrale)
    if (msg->data && !button_pressed_) {
      RCLCPP_INFO(this->get_logger(), ">>> Pulsante Falcon PREMUTO: Controllo drone ATTIVATO");
    } else if (!msg->data && button_pressed_) {
      RCLCPP_INFO(this->get_logger(), "<<< Pulsante Falcon RILASCIATO: Controllo drone DISATTIVATO");
      current_pov_vel_ = {0.0, 0.0, 0.0, 0.0};
    }
    button_pressed_ = msg->data;
  }

  void buttons_cb(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
  {
    // Aggiorna lo stato di tutti i bottoni
    for (size_t i = 0; i < msg->data.size() && i < button_states_.size(); ++i) {
      button_states_[i] = msg->data[i];
    }
    // Log alla pressione/rilascio di bottoni non-centrali
    RCLCPP_DEBUG(
      this->get_logger(),
      "Button states: [%d, %d, %d, %d]",
      button_states_[0], button_states_[1], button_states_[2], button_states_[3]);
  }

  void peg_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    peg_pos_enu_[0] = msg->pose.position.x;
    peg_pos_enu_[1] = msg->pose.position.y;
    peg_pos_enu_[2] = msg->pose.position.z;
    auto & q = msg->pose.orientation;
    peg_yaw_enu_ = std::atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z));
  }

  void actual_pov_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() >= 4) {
      actual_pov_[0] = msg->data[0]; // Xc
      actual_pov_[1] = msg->data[1]; // Yc
      actual_pov_[2] = msg->data[2]; // Zc
      actual_pov_[3] = msg->data[3]; // Pan
    }

    // Se il pulsante non è premuto, il riferimento segue la posizione reale del drone
    if (!button_pressed_) {
      current_pov_ref_[0] = actual_pov_[0];
      current_pov_ref_[1] = actual_pov_[1];
      current_pov_ref_[2] = actual_pov_[2];
      current_pov_ref_[3] = actual_pov_[3];
      current_pov_vel_ = {0.0, 0.0, 0.0, 0.0};
    }
  }

  /**
   * Calcola la forza repulsiva data la distanza dal bordo e la distanza massima.
   * Forza = 0 se dist > d_activation
   * Forza cresce quadraticamente da 0 a max quando dist va da d_activation a 0
   */
  double repulsive_force(double dist, double d_max, double activation_ratio, double k_rep, double alpha, double max_rep)
  {
    double d_activation = d_max * activation_ratio;
    
    if (dist >= d_activation || dist <= 0.0) {
      // Se siamo oltre la zona di attivazione: nessuna forza
      // Se dist <= 0: vincolo già violato, forza massima
      if (dist <= 0.0) return max_rep;
      return 0.0;
    }
    
    // Funzione Esponenziale: F = k_rep * (exp(alpha * normalized) - 1)
    // alpha controlla la "durezza" della curva 
    double normalized = (d_activation - dist) / d_activation; // 0..1
    double force = k_rep * (std::exp(alpha * normalized) - 1.0);
    
    return std::min(force, max_rep);
  }

  void control_loop()
  {
    double k = this->get_parameter("k_spring").as_double();
    double alpha = this->get_parameter("alpha").as_double();
    double b = this->get_parameter("b_damping").as_double();
    double max_f = this->get_parameter("max_force").as_double();
    double deadband = this->get_parameter("deadband").as_double();
    double dt = this->get_parameter("dt").as_double();

    // Calcola velocità istantanea filtrata dell' haptic per lo smorzamento viscoso
    if (first_pose_received_) {
      for (int i = 0; i < 3; ++i) {
        double raw_vel = (falcon_pos_[i] - prev_falcon_pos_[i]) / dt;
        falcon_vel_[i] = 0.8 * falcon_vel_[i] + 0.2 * raw_vel; // Filtro passa-basso di 1° ordine
        //falcon_vel_[i] = raw_vel;
        prev_falcon_pos_[i] = falcon_pos_[i];
      }
    }

    // Parametri campo potenziale
    double fov_h_deg = this->get_parameter("fov_h").as_double();
    double fov_v_deg = this->get_parameter("fov_v").as_double();
    double x_min = this->get_parameter("x_min_safety").as_double();
    double k_rep = this->get_parameter("k_repulsive").as_double();
    double act_ratio = this->get_parameter("activation_ratio").as_double();
    double act_ratio_cam = this->get_parameter("activation_ratio_cam").as_double();
    double max_rep = this->get_parameter("max_repulsive_force").as_double();

    double T_h = std::tan(fov_h_deg * M_PI / 360.0); // tan(fov_h/2)
    double T_v = std::tan(fov_v_deg * M_PI / 360.0); // tan(fov_v/2)

    // Stato attuale della camera (dal topic /actual_pov)
    double Xc = actual_pov_[0];
    double Yc = actual_pov_[1];
    double Zc = actual_pov_[2];

    // =====================================================
    // CALCOLO DISTANZE DAI BORDI CAMERA
    // =====================================================
    // I vincoli nel frame camera sono:
    //   Bordo Destro:    Yc <= +T_h * Xc  ->  dist = T_h*Xc - Yc
    //   Bordo Sinistro:  Yc >= -T_h * Xc  ->  dist = T_h*Xc + Yc
    //   Bordo Alto:      Zc <= +T_v * Xc  ->  dist = T_v*Xc - Zc
    //   Bordo Basso:     Zc >= -T_v * Xc  ->  dist = T_v*Xc + Zc
    //   Dist. Sicurezza: Xc >= X_min      ->  dist = Xc - X_min

    double half_width = T_h * Xc;   // semi-larghezza FOV a distanza Xc
    double half_height = T_v * Xc;  // semi-altezza FOV a distanza Xc

    double dist_right  = half_width - Yc;
    double dist_left   = half_width + Yc;
    double dist_top    = half_height - Zc;
    double dist_bottom = half_height + Zc;
    double dist_safety = Xc - x_min;

    // =====================================================
    // CALCOLO FORZE REPULSIVE
    // =====================================================
    // Forza sull'asse Y del Falcon (comanda Pan/Orbit):
    //   - Bordo destro violato -> forza che spinge Falcon Y verso sinistra (negativa)
    //   - Bordo sinistro violato -> forza che spinge Falcon Y verso destra (positiva)
    double f_rep_right = repulsive_force(dist_right, half_width, act_ratio_cam, k_rep, alpha, max_rep);
    double f_rep_left  = repulsive_force(dist_left,  half_width, act_ratio_cam, k_rep, alpha, max_rep);
    double f_rep_y = -f_rep_right + f_rep_left; // Forza netta su asse Y Falcon

    // Forza sull'asse Z del Falcon (comanda Zc/Altezza):
    //   - Bordo alto violato -> forza che spinge Falcon Z verso l'alto (positiva)
    //   - Bordo basso violato -> forza che spinge Falcon Z verso il basso (negativa)
    double f_rep_top    = repulsive_force(dist_top,    half_height, act_ratio_cam, k_rep, alpha*2, max_rep);
    double f_rep_bottom = repulsive_force(dist_bottom, half_height, act_ratio_cam, k_rep, alpha*2, max_rep);
    double f_rep_z = f_rep_top - f_rep_bottom; // Netto su asse Z Falcon

    // Forza sull'asse X del Falcon (comanda Xc/Zoom):
    //   - Troppo vicino all'oggetto -> forza POSITIVA che spinge Falcon X in avanti
    //     (Falcon X positivo -> Xc aumenta -> drone si allontana dall'oggetto)
    double f_rep_safety = repulsive_force(dist_safety, Xc, act_ratio, k_rep, alpha, max_rep);
    double f_rep_x = +f_rep_safety; // Spinge LONTANO dall'oggetto

    // =====================================================
    // FORZA TOTALE = Molla centering ammortizzata + Campo Potenziale
    // =====================================================
    std::vector<double> forces(3, 0.0);
    for (int i = 0; i < 3; ++i) {
      // Forza elastica ammortizzata: F = -k * x - b * v
      forces[i] = -k * falcon_pos_[i] - b * falcon_vel_[i];
    }
    
    // Somma le forze repulsive (solo quando il pulsante è premuto)
    if (button_pressed_) {
      forces[0] += f_rep_x;
      forces[1] += f_rep_y;
      forces[2] += f_rep_z;
    }

    // Saturazione di sicurezza
    for (int i = 0; i < 3; ++i) {
      forces[i] = std::max(-max_f, std::min(max_f, forces[i]));
    }

    // Pubblica forza
    auto force_msg = std_msgs::msg::Float64MultiArray();
    force_msg.data = forces;
    force_pub_->publish(force_msg);

    // 2. Integrazione PoV se il pulsante è premuto
    if (button_pressed_) {
      // Mappatura assi Falcon -> Comandi
      // Falcon X (Avanti/Dietro)   -> Xc (Zoom)
      // Falcon Y (Destra/Sinistra) -> Pan (Orbit)
      // Falcon Z (Su/Giù)          -> Zc (Altezza)
      
      double joy_scale = this->get_parameter("joy_scale").as_double();
      double xc_cmd  = apply_deadband(falcon_pos_[0], deadband) * joy_scale; 
      double pan_cmd = apply_deadband(falcon_pos_[1], deadband) * joy_scale;  
      double zc_cmd  = -apply_deadband(falcon_pos_[2], deadband) * joy_scale;  

      double v_pan_max = this->get_parameter("v_pan_max").as_double();
      double v_zc_max  = this->get_parameter("v_zc_max").as_double();
      double v_xc_max  = this->get_parameter("v_xc_max").as_double();

      // Calcolo velocità di riferimento (interpreto posizione haptic come velocità)
      current_pov_vel_[0] = xc_cmd  * v_xc_max;
      current_pov_vel_[1] = 0.0; // Yc non comandata direttamente
      current_pov_vel_[2] = zc_cmd  * v_zc_max;
      current_pov_vel_[3] = pan_cmd * v_pan_max;

      // Integrazione posizione (SENZA limiti di sicurezza: li gestisce l'MPC + feedback)
      current_pov_ref_[0] += current_pov_vel_[0] * dt; // Xc (Zoom)
      current_pov_ref_[2] += current_pov_vel_[2] * dt; // Zc
      current_pov_ref_[3] += current_pov_vel_[3] * dt; // Pan_mutuo
      
      // Wrap pan_mutuo
      current_pov_ref_[3] = std::fmod(current_pov_ref_[3] + M_PI, 2.0 * M_PI);
      if (current_pov_ref_[3] < 0) current_pov_ref_[3] += 2.0 * M_PI;
      current_pov_ref_[3] -= M_PI;

      // 3. Pubblicazione Goal (Legacy) - DISATTIVATA
      /*
      auto goal_msg = std_msgs::msg::Float64MultiArray();
      goal_msg.data = current_pov_ref_;
      goal_pub_->publish(goal_msg);
      */

      // 4. Pubblicazione Haptic Ref (Completo)
      auto haptic_msg = std_msgs::msg::Float64MultiArray();
      haptic_msg.data.insert(haptic_msg.data.end(), current_pov_ref_.begin(), current_pov_ref_.end());
      haptic_msg.data.insert(haptic_msg.data.end(), current_pov_vel_.begin(), current_pov_vel_.end());
      haptic_ref_pub_->publish(haptic_msg);
    } else {
      current_pov_vel_ = {0.0, 0.0, 0.0, 0.0};
    }

    // =====================================================
    // MODO PEG (bottone 2 = sopra): teleop drone peg nel frame camera MPC
    // Falcon X → profondità, Falcon Y → laterale, Falcon Z → quota
    // Bottone 1 → yaw CCW, Bottone 3 → yaw CW
    // =====================================================
    bool peg_btn = (button_states_[2] == 1);
    if (peg_btn && !peg_mode_active_) {  // fronte di salita: inizializza dal peg reale
      peg_target_pos_ = peg_pos_enu_;
      peg_target_yaw_ = peg_yaw_enu_;
    }
    peg_mode_active_ = peg_btn;

    if (peg_btn) {
      double joy_scale = this->get_parameter("joy_scale").as_double();
      double v_t  = this->get_parameter("v_xc_max").as_double();  // velocità trasl [m/s]
      v_t = 0.5 * v_t;
      double v_z  = this->get_parameter("v_zc_max").as_double();
      double v_yr = this->get_parameter("v_pan_max").as_double(); // velocità yaw [rad/s]
      double psi_body = peg_target_yaw_;  // Usa lo yaw del peg drone (body frame)

      // L'hardware Falcon ha la X e la Y invertite rispetto al FLU (avanti/sinistra).
      // Matematicamente, questo equivale a una rotazione di 180 gradi attorno all'asse Z
      Eigen::Matrix3d R_falcon_to_body;
      R_falcon_to_body << -1.0,  0.0,  0.0,
                           0.0, -1.0,  0.0,
                           0.0,  0.0,  1.0;

      // Matrice dal Body del drone (FLU) al Mondo (ENU)
      Eigen::Matrix3d R_body_to_world;
      R_body_to_world << std::cos(psi_body), -std::sin(psi_body), 0.0,
                         std::sin(psi_body),  std::cos(psi_body), 0.0,
                         0.0,                 0.0,                1.0;

      // Vettore comandi grezzi (hardware frame)
      Eigen::Vector3d f_raw(
        apply_deadband(falcon_pos_[0], deadband) * joy_scale,
        apply_deadband(falcon_pos_[1], deadband) * joy_scale,
        apply_deadband(falcon_pos_[2], deadband) * joy_scale
      );

      // Catena cinematica completa: World = R_body_to_world * R_falcon_to_body * Falcon
      Eigen::Vector3d v_cmd = R_body_to_world * R_falcon_to_body * f_raw;

      peg_target_pos_[0] += v_cmd.x() * v_t * dt;
      peg_target_pos_[1] += v_cmd.y() * v_t * dt;
      peg_target_pos_[2] += v_cmd.z() * v_z * dt;

      if (button_states_[1] == 1) peg_target_yaw_ += v_yr * dt;  // CCW
      if (button_states_[3] == 1) peg_target_yaw_ -= v_yr * dt;  // CW
      peg_target_yaw_ = std::fmod(peg_target_yaw_ + M_PI, 2.0*M_PI);
      if (peg_target_yaw_ < 0.0) peg_target_yaw_ += 2.0*M_PI;
      peg_target_yaw_ -= M_PI;

      geometry_msgs::msg::PoseStamped peg_msg;
      peg_msg.header.stamp = this->now();
      peg_msg.header.frame_id = "map";
      peg_msg.pose.position.x = peg_target_pos_[0];
      peg_msg.pose.position.y = peg_target_pos_[1];
      peg_msg.pose.position.z = peg_target_pos_[2];
      peg_msg.pose.orientation.w = std::cos(peg_target_yaw_ * 0.5);
      peg_msg.pose.orientation.z = std::sin(peg_target_yaw_ * 0.5);
      peg_live_pub_->publish(peg_msg);
    }
  }

  double apply_deadband(double val, double deadband)
  {
    if (std::abs(val) < deadband) return 0.0;
    return (val > 0) ? (val - deadband) : (val + deadband);
  }

  // ROS 2 objects
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr peg_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr button_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr buttons_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr actual_pov_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr force_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr goal_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr haptic_ref_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr peg_live_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State — MODO MPC
  std::vector<double> current_pov_ref_;
  std::vector<double> current_pov_vel_;
  std::vector<double> actual_pov_;
  std::vector<double> falcon_pos_;
  std::vector<double> prev_falcon_pos_;
  std::vector<double> falcon_vel_;
  bool first_pose_received_;
  bool button_pressed_;            // bottone 0 (centrale)
  std::vector<int> button_states_; // [btn0..btn3]

  // State — MODO PEG
  std::vector<double> peg_pos_enu_;    // posizione reale peg (da /peg_ref_pose)
  std::vector<double> peg_target_pos_; // setpoint integrato
  double peg_yaw_enu_;                 // yaw reale peg
  double peg_target_yaw_;              // setpoint yaw integrato
  bool peg_mode_active_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FDHapticJoyNode>());
  rclcpp::shutdown();
  return 0;
}
