// Copyright 2022, ICube Laboratory, University of Strasbourg
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <Eigen/Dense>

#include <stddef.h>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "fd_clutch_broadcaster/fd_clutch_broadcaster.hpp"

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcpputils/split.hpp"
#include "rcutils/logging_macros.h"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"


namespace rclcpp_lifecycle
{
class State;
}  // namespace rclcpp_lifecycle

namespace fd_clutch_broadcaster
{

FdClutchBroadcaster::FdClutchBroadcaster() {}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
FdClutchBroadcaster::on_init()
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
FdClutchBroadcaster::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration FdClutchBroadcaster::state_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Register all 4 button state interfaces
  for (const auto & name : button_interface_names_) {
    if (!name.empty()) {
      state_interfaces_config.names.push_back(name);
    }
  }
  return state_interfaces_config;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
FdClutchBroadcaster::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Declare parameters
  try {
    auto_declare<std::string>("clutch_interface_name", std::string("button0/position"));
    auto_declare<bool>("is_interface_a_button", true);
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during configure stage with message: %s \n", e.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // Build the 4 button interface names: button0/position .. button3/position
  button_interface_names_.clear();
  for (int i = 0; i < 4; ++i) {
    button_interface_names_.push_back("button" + std::to_string(i) + "/position");
  }
  // Keep legacy name pointing to button0 for backward compat
  clutch_interface_name_ = button_interface_names_[0];

  try {
    // Legacy Bool publisher on /fd/button_state (button 0 only, backward compat)
    clutch_publisher_ = get_node()->create_publisher<std_msgs::msg::Bool>(
      "/fd/button_state",
      rclcpp::SystemDefaultsQoS());
    realtime_clutch_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Bool>>(
      clutch_publisher_);

    // New publisher: all 4 buttons as Int32MultiArray on /fd/button_states
    buttons_publisher_ = get_node()->create_publisher<std_msgs::msg::Int32MultiArray>(
      "/fd/button_states",
      rclcpp::SystemDefaultsQoS());
    realtime_buttons_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Int32MultiArray>>(
      buttons_publisher_);
    // Pre-allocate 4 elements in the message
    realtime_buttons_publisher_->msg_.data.resize(4, 0);
  } catch (const std::exception & e) {
    // get_node() may throw, logging raw here
    fprintf(stderr, "Exception thrown during configure stage with message: %s \n", e.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
FdClutchBroadcaster::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (state_interfaces_.size() != 1) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Expecting exactly one state interface.");
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
FdClutchBroadcaster::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::return_type FdClutchBroadcaster::update(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  RCLCPP_DEBUG(get_node()->get_logger(), "Entering update()");

  const size_t n_buttons = state_interfaces_.size(); // should be 4

  // --- Publish /fd/button_states (Int32MultiArray, all buttons) ---
  if (realtime_buttons_publisher_ && realtime_buttons_publisher_->trylock()) {
    auto & data = realtime_buttons_publisher_->msg_.data;
    data.resize(n_buttons, 0);
    for (size_t i = 0; i < n_buttons; ++i) {
      double val = state_interfaces_[i].get_value();
      data[i] = (val > 0.5) ? 1 : 0;
    }
    realtime_buttons_publisher_->unlockAndPublish();
  }

  // --- Publish /fd/button_state (Bool, button 0 only — backward compat) ---
  if (realtime_clutch_publisher_ && realtime_clutch_publisher_->trylock()) {
    RCLCPP_DEBUG(get_node()->get_logger(), "Lock acquired");
    double read_value = (n_buttons > 0) ? state_interfaces_[0].get_value() : 0.0;
    realtime_clutch_publisher_->msg_.data = (read_value > 0.5);
    RCLCPP_DEBUG(get_node()->get_logger(), "publish and unlock");
    realtime_clutch_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace fd_clutch_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  fd_clutch_broadcaster::FdClutchBroadcaster, controller_interface::ControllerInterface)
