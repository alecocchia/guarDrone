#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"

using std::placeholders::_1;

class FDHapticJoyNode : public rclcpp::Node
{
public:
  FDHapticJoyNode()
  : Node("fd_haptic_joy_node")
  {
    // Parametri molla aptica
    this->declare_parameter("k_spring", 40.0);    // N/m
    this->declare_parameter("max_force", 10.0);    // N
    this->declare_parameter("deadband", 0.005);    // metri
    this->declare_parameter("joy_scale", 15.0);    // 0.05m -> 0.75
    
    // Parametri integrazione PoV (stessi di human_goal_node)
    this->declare_parameter("v_pan_max", 0.5);   
    this->declare_parameter("v_zc_max", 0.5);  
    this->declare_parameter("v_xc_max", 1.0);
    this->declare_parameter("dt", 0.02);           // 50 Hz

    // Subscribers
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/fd/ee_pose", 10, std::bind(&FDHapticJoyNode::pose_cb, this, _1));
    
    button_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/fd/button_state", 10, std::bind(&FDHapticJoyNode::button_cb, this, _1));

    actual_pov_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/actual_pov", 10, std::bind(&FDHapticJoyNode::actual_pov_cb, this, _1));

    // Publishers
    force_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/fd/fd_controller/commands", 10);
    goal_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pov_target", 10);

    // Initial state
    current_pov_ref_ = {2.0, 0.0, 0.0, 0.0}; // [Xc, Yc, Zc, Pan_mutuo]
    falcon_pos_ = {0.0, 0.0, 0.0};
    button_pressed_ = false;

    // Timer loop a 50Hz
    double dt = this->get_parameter("dt").as_double();
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(dt), std::bind(&FDHapticJoyNode::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "Haptic Joy Node avviato. Premi il pulsante per comandare il drone.");
  }

private:
  void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    falcon_pos_[0] = msg->pose.position.x;
    falcon_pos_[1] = msg->pose.position.y;
    falcon_pos_[2] = msg->pose.position.z;
  }

  void button_cb(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data && !button_pressed_) {
      RCLCPP_INFO(this->get_logger(), ">>> Pulsante Falcon PREMUTO: Controllo drone ATTIVATO");
    } else if (!msg->data && button_pressed_) {
      RCLCPP_INFO(this->get_logger(), "<<< Pulsante Falcon RILASCIATO: Controllo drone DISATTIVATO");
    }
    button_pressed_ = msg->data;
  }

  void actual_pov_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    // Se il pulsante non è premuto, il riferimento segue la posizione reale del drone
    if (!button_pressed_ && msg->data.size() >= 4) {
      current_pov_ref_[0] = msg->data[0]; // Xc
      current_pov_ref_[1] = msg->data[1]; // Yc
      current_pov_ref_[2] = msg->data[2]; // Zc
      current_pov_ref_[3] = msg->data[3]; // Pan
    }
  }

  void control_loop()
  {
    double k = this->get_parameter("k_spring").as_double();
    double max_f = this->get_parameter("max_force").as_double();
    double deadband = this->get_parameter("deadband").as_double();
    double dt = this->get_parameter("dt").as_double();

    // Calcolo forza della molla (sempre attiva per dare feedback)
    // Usiamo lo zero come centro di riposo
    std::vector<double> forces(3, 0.0);
    for (int i = 0; i < 3; ++i) {
      // Forza elastica: F = -k * x
      forces[i] = -k * falcon_pos_[i];
      
      // Saturazione per sicurezza
      if (forces[i] > max_f) forces[i] = max_f;
      if (forces[i] < -max_f) forces[i] = -max_f;
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
      double xc_cmd  = apply_deadband(-falcon_pos_[0], deadband) * joy_scale; // Avanti -> Avvicina
      double pan_cmd = apply_deadband(falcon_pos_[1], deadband) * joy_scale;  // Destra -> Pan positivo
      double zc_cmd  = apply_deadband(falcon_pos_[2], deadband) * joy_scale;  // Su -> Alza

      double v_pan_max = this->get_parameter("v_pan_max").as_double();
      double v_zc_max  = this->get_parameter("v_zc_max").as_double();
      double v_xc_max  = this->get_parameter("v_xc_max").as_double();

      // Integrazione
      current_pov_ref_[0] -= xc_cmd  * v_xc_max  * dt; // Xc (Zoom)
      current_pov_ref_[3] += pan_cmd * v_pan_max * dt; // Pan_mutuo
      current_pov_ref_[2] += zc_cmd  * v_zc_max  * dt; // Zc

      // Limiti di sicurezza
      current_pov_ref_[0] = std::max(1.5, std::min(8.0, current_pov_ref_[0])); // Xc
      current_pov_ref_[2] = std::max(-0.5, std::min(0.5, current_pov_ref_[2])); // Zc
      
      // Wrap pan_mutuo
      current_pov_ref_[3] = std::fmod(current_pov_ref_[3] + M_PI, 2.0 * M_PI);
      if (current_pov_ref_[3] < 0) current_pov_ref_[3] += 2.0 * M_PI;
      current_pov_ref_[3] -= M_PI;

      // 3. Pubblicazione Goal al drone (SOLO se il pulsante è premuto)
      auto goal_msg = std_msgs::msg::Float64MultiArray();
      goal_msg.data = current_pov_ref_;
      goal_pub_->publish(goal_msg);
    }
  }

  double apply_deadband(double val, double deadband)
  {
    if (std::abs(val) < deadband) return 0.0;
    return (val > 0) ? (val - deadband) : (val + deadband);
  }

  // State
  std::vector<double> current_pov_ref_;
  std::vector<double> falcon_pos_;
  bool button_pressed_;

  // ROS 2 objects
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr button_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr actual_pov_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr force_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr goal_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FDHapticJoyNode>());
  rclcpp::shutdown();
  return 0;
}
