#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <thread>
#include <chrono>
#include "takeoff_action.hpp"
#include "cbf_action.hpp"
#include "run_mpc_action.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    // ── 1. Nodo ROS2 condiviso ────────────────────────────────────
    auto ros_node = std::make_shared<rclcpp::Node>("bt_executor");

    // ── 2. Factory ────────────────────────────────────────────────
    BT::BehaviorTreeFactory factory;

    // ── 3. Registra TakeOffAction ─────────────────────────────────
    // In v3 non si può passare parametri extra a registerNodeType
    // Si usa registerBuilder con una lambda che cattura ros_node
    factory.registerBuilder<TakeOffAction>(
        "TakeOffAction",
        [ros_node](const std::string & name,
                   const BT::NodeConfiguration & config)
        {
            return std::make_unique<TakeOffAction>(name, config, ros_node);
        });

    factory.registerBuilder<CbfAction>(
    "CbfAction",
    [ros_node](const std::string & name,
               const BT::NodeConfiguration & config)
    {
        return std::make_unique<CbfAction>(name, config, ros_node);
    });    

    factory.registerBuilder<RunMPCOffboardTask>(
    "RunMPCOffboardTask",
    [ros_node](const std::string & name,
               const BT::NodeConfiguration & config)
    {
        return std::make_unique<RunMPCOffboardTask>(name, config, ros_node);
    });

    // ── 4. Carica XML ─────────────────────────────────────────────
    std::string pkg_path =
        ament_index_cpp::get_package_share_directory("dop_drones_take_off");
    std::string xml_path = pkg_path + "/drone_bt.xml";

    RCLCPP_INFO(ros_node->get_logger(), "Carico BT da: %s", xml_path.c_str());
    auto tree = factory.createTreeFromFile(xml_path);

    // ── 5. Logger ─────────────────────────────────────────────────
    BT::StdCoutLogger logger(tree);

    // ── 6. Loop principale ────────────────────────────────────────
    // In v3 il metodo si chiama tickRoot(), non tickOnce()
    BT::NodeStatus status = BT::NodeStatus::RUNNING;

    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
        status = tree.tickRoot();              // ← v3: tickRoot()
        rclcpp::spin_some(ros_node);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (status == BT::NodeStatus::SUCCESS)
        RCLCPP_INFO(ros_node->get_logger(), ">>> BT: SUCCESS <<<");
    else
        RCLCPP_ERROR(ros_node->get_logger(), ">>> BT: FAILURE <<<");

    rclcpp::shutdown();
    return 0;
}