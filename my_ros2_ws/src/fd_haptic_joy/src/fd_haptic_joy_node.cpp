#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

using std::placeholders::_1;

#include "geometry_msgs/msg/quaternion.hpp"

// Funzione per convertire RPY in Quaternione ROS usando Eigen
geometry_msgs::msg::Quaternion rpy_to_quaternion(double roll, double pitch,
                                                 double yaw) {
  // Eigen usa la convenzione standard per combinare le rotazioni (Yaw * Pitch *
  // Roll)
  Eigen::Quaterniond q_eigen =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

  geometry_msgs::msg::Quaternion q_ros;
  q_ros.w = q_eigen.w();
  q_ros.x = q_eigen.x();
  q_ros.y = q_eigen.y();
  q_ros.z = q_eigen.z();

  return q_ros;
}

class FDHapticJoyNode : public rclcpp::Node {
public:
  FDHapticJoyNode() : Node("fd_haptic_joy_node") {
    // Parametri molla aptica
    this->declare_parameter("k_spring",
                            50.0); // Aumentato per un ritorno più forte (da 40)
    this->declare_parameter(
        "b_damping", 10.0); // Coefficiente di smorzamento viscoso virtuale
    this->declare_parameter("max_force", 15.0); // Limite hardware
    this->declare_parameter("deadband", 0.005);
    this->declare_parameter("joy_scale", 15.0);

    // Parametri integrazione PoV sferico
    this->declare_parameter("v_r_max", 1.2);    // velocità radiale max [m/s]
    this->declare_parameter("v_beta_max", 0.5); // velocità azimut max [rad/s]
    this->declare_parameter("v_gamma_max",
                            0.5); // velocità elevazione max [rad/s]
    this->declare_parameter("dt", 0.01);

    // Parametri Campo Potenziale (Force Feedback dai vincoli MPC)
    this->declare_parameter("fov_h", 80.0);
    this->declare_parameter("fov_v", 60.0);
    this->declare_parameter("r_min_safety",
                            1.5); // distanza minima di sicurezza [m]
    this->declare_parameter("k_repulsive", 1.0);
    this->declare_parameter("alpha", 3.0);
    this->declare_parameter("activation_ratio", 1.0);
    this->declare_parameter("activation_ratio_cam", 1.0);
    this->declare_parameter("max_repulsive_force", 15.0);

    this->declare_parameter("v_pan_max", 0.5);
    this->declare_parameter("v_zc_max", 0.5);
    this->declare_parameter("v_xc_max", 1.0);
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

    // Subscriber: bottone singolo /fd/button_state (Bool, backward compat —
    // bottone centrale)
    button_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/fd/button_state", 10,
        std::bind(&FDHapticJoyNode::button_cb, this, _1));

    // Subscriber: tutti e 4 i bottoni /fd/button_states (Int32MultiArray)
    buttons_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
        "/fd/button_states", 10,
        std::bind(&FDHapticJoyNode::buttons_cb, this, _1));

    actual_pov_sub_ =
        this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/actual_pov", 10,
            std::bind(&FDHapticJoyNode::actual_pov_cb, this, _1));
    peg_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/peg_ref_pose", 10,
        std::bind(&FDHapticJoyNode::peg_pose_cb, this, _1));

    // Publishers
    force_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/fd/fd_controller/commands", 10);
    haptic_ref_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/haptic_ref", 10);
    peg_live_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/peg_live_pose", 10);

    // Initial state
    // Stato PoV in coordinate sferiche: [r, beta, gamma]
    // r     = distanza 3D dall'oggetto [m]
    // beta  = azimut nel piano XY [rad]  (orbita orizzontale)
    // gamma = elevazione dal piano XY [rad]  (0=piano, +pi/2=zenit)
    current_pov_ref_ = {3.0, M_PI, 0.0}; // default: 3m dietro, stessa quota
    current_pov_vel_ = {0.0, 0.0, 0.0};  // [dr, d_beta, d_gamma]
    actual_pov_ = {3.0, M_PI, 0.0, 0.0}; // [r, beta, gamma, yaw_err] dal drone
    falcon_pos_ = {0.0, 0.0, 0.0};
    prev_falcon_pos_ = {0.0, 0.0, 0.0};
    falcon_vel_ = {0.0, 0.0, 0.0};
    first_pose_received_ = false;
    button_pressed_ = false;
    button_states_ = {0, 0, 0, 0}; // stato corrente dei 4 bottoni [btn0..btn3]
    peg_pos_enu_ = {0.0, 0.0, 0.0};
    peg_target_pos_ = {0.0, 0.0, 0.0};
    peg_target_yaw_ = 0.0;
    peg_yaw_enu_ = 0.0;
    peg_mode_active_ = false;

    // Timer loop a 100Hz
    double dt = this->get_parameter("dt").as_double();
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(dt),
        std::bind(&FDHapticJoyNode::control_loop, this));

    RCLCPP_INFO(
        this->get_logger(),
        "Haptic Joy Node avviato con Force Feedback a Campo Potenziale.");
  }

private:
  void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
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

  void button_cb(const std_msgs::msg::Bool::SharedPtr msg) {
    // Backward compat: bottone 0 (quello centrale)
    if (msg->data && !button_pressed_) {
      RCLCPP_INFO(this->get_logger(),
                  ">>> Pulsante Falcon PREMUTO: Controllo drone ATTIVATO");
    } else if (!msg->data && button_pressed_) {
      RCLCPP_INFO(
          this->get_logger(),
          "<<< Pulsante Falcon RILASCIATO: Controllo drone DISATTIVATO");
      current_pov_vel_ = {0.0, 0.0, 0.0};
    }
    button_pressed_ = msg->data;
    if (!msg->data) {
      current_pov_vel_ = {0.0, 0.0, 0.0};
    }
  }

  void buttons_cb(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    // Aggiorna lo stato di tutti i bottoni
    for (size_t i = 0; i < msg->data.size() && i < button_states_.size(); ++i) {
      button_states_[i] = msg->data[i];
    }
    // Log alla pressione/rilascio di bottoni non-centrali
    RCLCPP_DEBUG(this->get_logger(), "Button states: [%d, %d, %d, %d]",
                 button_states_[0], button_states_[1], button_states_[2],
                 button_states_[3]);
  }

  void peg_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    peg_pos_enu_[0] = msg->pose.position.x;
    peg_pos_enu_[1] = msg->pose.position.y;
    peg_pos_enu_[2] = msg->pose.position.z;
    auto &q = msg->pose.orientation;
    peg_yaw_enu_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  void actual_pov_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    // Formato atteso: [r, beta, gamma, yaw_err]
    if (msg->data.size() >= 3) {
      actual_pov_[0] = msg->data[0]; // r
      actual_pov_[1] = msg->data[1]; // beta
      actual_pov_[2] = msg->data[2]; // gamma
    }
    if (msg->data.size() >= 4) {
      actual_pov_[3] = msg->data[3]; // yaw_err
    }

    // Se il pulsante non è premuto, il riferimento segue la posizione reale del
    // drone
    if (!button_pressed_) {
      current_pov_ref_[0] = actual_pov_[0]; // r
      current_pov_ref_[1] = actual_pov_[1]; // beta
      current_pov_ref_[2] = actual_pov_[2]; // gamma
      current_pov_vel_ = {0.0, 0.0, 0.0};
    }
  }

  /**
   * Calcola la forza repulsiva data la distanza dal bordo e la distanza
   * massima. Forza = 0 se dist > d_activation Forza cresce esponenzialmente da
   * 0 a max quando dist va da d_activation a 0
   */
  // dist = distanza attuale dal vincolo
  // d_max = distanza massima ammessa prima che la forza repulsiva cominci
  // (potenzialmente, senza contare activation_ratio) activation_ratio =
  // rapporto tra distanza di attivazione e distanza massima
  // --> d_activation = d_max * activation_ratio - Questa è la distanza dal
  // vincolo a cui viene effettivamente attivata la forza repulsiva k_rep =
  // guadagno della forza repulsiva alpha = esponente della funzione
  // esponenziale max_rep = forza repulsiva massima
  double repulsive_force(double dist, double d_max, double activation_ratio,
                         double k_rep, double alpha, double max_rep) {
    // d_activation è la distanza effettiva dal vincolo a cui la forza repulsiva
    // inizia ad agire
    double d_activation = d_max * activation_ratio;

    if (dist >= d_activation) {
      // Se siamo oltre la zona di attivazione: nessuna forza
      // Se dist <= 0: vincolo già violato, forza massima
      // if (dist <= 0.0) return max_rep;
      return 0.0;
    }

    // Funzione Esponenziale: F = k_rep * (exp(alpha * normalized) - 1)
    // alpha controlla la "durezza" della curva
    double normalized = (d_activation - dist) / d_activation; // 0..1
    double force = k_rep * (std::exp(alpha * normalized) - 1.0);

    return std::min(force, max_rep);
  }

  void control_loop() {
    double k = this->get_parameter("k_spring").as_double();
    double alpha = this->get_parameter("alpha").as_double();
    double b = this->get_parameter("b_damping").as_double();
    double max_f = this->get_parameter("max_force").as_double();
    double deadband = this->get_parameter("deadband").as_double();
    double dt = this->get_parameter("dt").as_double();

    // Calcola velocità istantanea filtrata dell' haptic per lo smorzamento
    // viscoso
    if (first_pose_received_) {
      for (int i = 0; i < 3; ++i) {
        double raw_vel = (falcon_pos_[i] - prev_falcon_pos_[i]) / dt;
        falcon_vel_[i] = 0.8 * falcon_vel_[i] +
                         0.2 * raw_vel; // Filtro passa-basso di 1° ordine
        // falcon_vel_[i] = raw_vel;
        prev_falcon_pos_[i] = falcon_pos_[i];
      }
    }

    // Parametri campo potenziale
    // =====================================================
    // CAMPO POTENZIALE: forza repulsiva basata su distanza radiale r
    // =====================================================
    double r_min = this->get_parameter("r_min_safety").as_double();
    double k_rep = this->get_parameter("k_repulsive").as_double();
    double act_ratio = this->get_parameter("activation_ratio").as_double();
    double act_ratio_cam =
        this->get_parameter("activation_ratio_cam").as_double();
    double max_rep = this->get_parameter("max_repulsive_force").as_double();

    // Parametri Campo Visivo (FoV)
    double fov_h = this->get_parameter("fov_h").as_double();
    double fov_v = this->get_parameter("fov_v").as_double();
    double limit_gamma_max = (fov_v / 2.0) * (M_PI / 180.0);
    double limit_yaw_max = (fov_h / 2.0) * (M_PI / 180.0);

    // 1. Asse X: Forza repulsiva basata su distanza radiale r
    double r_actual = actual_pov_[0]; // r [m]
    double dist_r_min =
        r_actual - r_min; // distanza attuale dal bordo del vincolo, >0 se
                          // sicuro, <0 se violato
    double f_rep_x = +repulsive_force(dist_r_min, r_min, act_ratio, k_rep,
                                      alpha / 2, max_rep);

    // 2. Asse Z: Forza repulsiva basata su elevazione gamma (limite FoV
    // verticale)
    double gamma_act = actual_pov_[2];
    double dist_gamma_top = limit_gamma_max - gamma_act;
    double dist_gamma_bot = gamma_act - (-limit_gamma_max);
    double f_rep_z_top = -repulsive_force(dist_gamma_top, limit_gamma_max,
                                          act_ratio_cam, k_rep, alpha, max_rep);
    double f_rep_z_bot = +repulsive_force(dist_gamma_bot, limit_gamma_max,
                                          act_ratio_cam, k_rep, alpha, max_rep);
    double f_rep_z = f_rep_z_top + f_rep_z_bot;

    // 3. Asse Y: Forza repulsiva basata su yaw_err (limite FoV orizzontale)
    double yaw_err_act = actual_pov_[3];
    double dist_yaw = limit_yaw_max - std::abs(yaw_err_act);
    double f_rep_yaw_mag = repulsive_force(
        dist_yaw, limit_yaw_max, act_ratio_cam, k_rep, alpha, max_rep);
    // Se yaw_err > 0, opponiamo una forza per spingere l'utente a correggere
    double f_rep_y = (yaw_err_act > 0) ? -f_rep_yaw_mag : f_rep_yaw_mag;

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

    // =====================================================
    // INTEGRAZIONE PoV sferico se il pulsante è premuto
    // Mapping assi Falcon -> Coordinate Sferiche:
    //   Falcon X (Avanti/Dietro)   -> dr     (zoom: avvicina/allontana)
    //   Falcon Y (Destra/Sinistra) -> d_beta (orbita orizzontale)
    //   Falcon Z (Su/Giù)          -> d_gamma(orbita verticale / elevazione)
    // =====================================================
    if (button_pressed_) {
      double joy_scale = this->get_parameter("joy_scale").as_double();
      double v_r_max = this->get_parameter("v_r_max").as_double();
      double v_beta_max = this->get_parameter("v_beta_max").as_double();
      double v_gamma_max = this->get_parameter("v_gamma_max").as_double();

      double dr_cmd =
          apply_deadband(falcon_pos_[0], deadband) * joy_scale * v_r_max;
      double dbeta_cmd =
          apply_deadband(falcon_pos_[1], deadband) * joy_scale * v_beta_max;
      double dgamma_cmd =
          apply_deadband(falcon_pos_[2], deadband) * joy_scale * v_gamma_max;

      current_pov_vel_[0] = dr_cmd;
      current_pov_vel_[1] = dbeta_cmd;
      current_pov_vel_[2] = dgamma_cmd;

      // Integrazione
      current_pov_ref_[0] += current_pov_vel_[0] * dt;          // r
      current_pov_ref_[0] = std::max(0.5, current_pov_ref_[0]); // r >= 0.5 m
      current_pov_ref_[1] += current_pov_vel_[1] * dt;          // beta
      current_pov_ref_[1] =
          std::fmod(current_pov_ref_[1] + M_PI, 2.0 * M_PI); // wrap
      if (current_pov_ref_[1] < 0)
        current_pov_ref_[1] += 2.0 * M_PI;
      current_pov_ref_[1] -= M_PI;
      current_pov_ref_[2] += current_pov_vel_[2] * dt; // gamma
      current_pov_ref_[2] =
          std::max(-M_PI / 2.0 + 0.05,
                   std::min(M_PI / 2.0 - 0.05, current_pov_ref_[2])); // clamp

      // Pubblica: [r, beta, gamma, dr, d_beta, d_gamma]  (6 valori)
      auto haptic_msg = std_msgs::msg::Float64MultiArray();
      haptic_msg.data.insert(haptic_msg.data.end(), current_pov_ref_.begin(),
                             current_pov_ref_.end());
      haptic_msg.data.insert(haptic_msg.data.end(), current_pov_vel_.begin(),
                             current_pov_vel_.end());
      haptic_ref_pub_->publish(haptic_msg);
    } else {
      current_pov_vel_ = {0.0, 0.0, 0.0};
    }

    // =====================================================
    // MODO PEG (bottone 2 = sopra): teleop drone peg nel frame camera MPC
    // Falcon X → profondità, Falcon Y → laterale, Falcon Z → quota
    // Bottone 1 → yaw CCW, Bottone 3 → yaw CW
    // =====================================================
    bool peg_btn = (button_states_[2] == 1);
    if (peg_btn &&
        !peg_mode_active_) { // fronte di salita: inizializza dal peg reale
      peg_target_pos_ = peg_pos_enu_;
      peg_target_yaw_ = peg_yaw_enu_;
    }
    peg_mode_active_ = peg_btn;

    if (peg_btn) {
      double joy_scale = this->get_parameter("joy_scale").as_double();
      double v_t =
          this->get_parameter("v_xc_max").as_double(); // velocità trasl [m/s]
      v_t = 0.5 * v_t;
      double v_z = this->get_parameter("v_zc_max").as_double();
      double v_yr =
          this->get_parameter("v_pan_max").as_double(); // velocità yaw [rad/s]
      double psi_body =
          peg_target_yaw_; // Usa lo yaw del peg drone (body frame)

      // L'hardware Falcon ha la X e la Y invertite rispetto al FLU
      // (avanti/sinistra). Matematicamente, questo equivale a una rotazione di
      // 180 gradi attorno all'asse Z
      Eigen::Matrix3d R_falcon_to_body;
      R_falcon_to_body << -1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0;

      // Matrice dal Body del drone (FLU) al Mondo (ENU)
      Eigen::Matrix3d R_body_to_world;
      R_body_to_world << std::cos(psi_body), -std::sin(psi_body), 0.0,
          std::sin(psi_body), std::cos(psi_body), 0.0, 0.0, 0.0, 1.0;

      // Vettore comandi grezzi (hardware frame)
      Eigen::Vector3d f_raw(
          apply_deadband(falcon_pos_[0], deadband) * joy_scale,
          apply_deadband(falcon_pos_[1], deadband) * joy_scale,
          apply_deadband(falcon_pos_[2], deadband) * joy_scale);

      // Catena cinematica completa: World = R_body_to_world * R_falcon_to_body
      // * Falcon
      Eigen::Vector3d v_cmd = R_body_to_world * R_falcon_to_body * f_raw;

      peg_target_pos_[0] += v_cmd.x() * v_t * dt;
      peg_target_pos_[1] += v_cmd.y() * v_t * dt;
      peg_target_pos_[2] += v_cmd.z() * v_z * dt;

      if (button_states_[1] == 1)
        peg_target_yaw_ += v_yr * dt; // CCW
      if (button_states_[3] == 1)
        peg_target_yaw_ -= v_yr * dt; // CW
      peg_target_yaw_ = std::fmod(peg_target_yaw_ + M_PI, 2.0 * M_PI);
      if (peg_target_yaw_ < 0.0)
        peg_target_yaw_ += 2.0 * M_PI;
      peg_target_yaw_ -= M_PI;

      geometry_msgs::msg::PoseStamped peg_msg;
      peg_msg.header.stamp = this->now();
      peg_msg.header.frame_id = "map";
      peg_msg.pose.position.x = peg_target_pos_[0];
      peg_msg.pose.position.y = peg_target_pos_[1];
      peg_msg.pose.position.z = peg_target_pos_[2];
      peg_msg.pose.orientation = rpy_to_quaternion(0.0, 0.0, peg_target_yaw_);
      peg_live_pub_->publish(peg_msg);
    }
  }

  double apply_deadband(double val, double deadband) {
    if (std::abs(val) < deadband)
      return 0.0;
    return (val > 0) ? (val - deadband) : (val + deadband);
  }

  // ROS 2 objects
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      peg_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr button_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr buttons_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
      actual_pov_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr force_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr goal_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      haptic_ref_pub_;
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

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FDHapticJoyNode>());
  rclcpp::shutdown();
  return 0;
}
