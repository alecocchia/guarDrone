#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

class TakeOffAction : public BT::StatefulActionNode
{
public:
    // v3 usa NodeConfiguration, non NodeConfig
    TakeOffAction(const std::string & name,
                  const BT::NodeConfiguration & config,
                  rclcpp::Node::SharedPtr ros_node)
    : BT::StatefulActionNode(name, config),
      ros_node_(ros_node)
    {
        start_pub_ = ros_node_->create_publisher<std_msgs::msg::Bool>(
            "/takeoff/start", 10);

        status_sub_ = ros_node_->create_subscription<std_msgs::msg::String>(
            "/takeoff/status", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                last_status_ = msg->data;
            });
    }

    // v3: stessa sintassi
    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus onStart() override
    {
        RCLCPP_INFO(ros_node_->get_logger(), "[TakeOffAction] Invio start...");
        std_msgs::msg::Bool msg;
        msg.data = true;
        start_pub_->publish(msg);
        last_status_ = "";
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        rclcpp::spin_some(ros_node_);

        if (last_status_ == "SUCCESS") {
            RCLCPP_INFO(ros_node_->get_logger(), "[TakeOffAction] SUCCESS");
            return BT::NodeStatus::SUCCESS;
        }
        if (last_status_ == "FAILURE") {
            RCLCPP_ERROR(ros_node_->get_logger(), "[TakeOffAction] FAILURE");
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        RCLCPP_WARN(ros_node_->get_logger(), "[TakeOffAction] Interrotto");
    }

private:
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr      start_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
    std::string last_status_ = "";
};