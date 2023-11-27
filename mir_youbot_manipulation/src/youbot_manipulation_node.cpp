/*
* Copyright 2023 Bonn-Rhein-Sieg University
*
* ROS2 authors: Jay Parikh, Kamendu Panchal, Chaitanya Gumudala, Mohsen Azizmalayeri.
*
*/

#include "mir_youbot_manipulation/youbot_manipulation_node.hpp"

ManipulatorRosNode::ManipulatorRosNode(const rclcpp::NodeOptions& options) : rclcpp_lifecycle::LifecycleNode("youbot_manipulation_node", options)
{
    RCLCPP_INFO(get_logger(), "Manipulator Node is created");
    auto overrides = this->get_node_options().parameter_overrides();
    for (auto& override : overrides) {
        declare_parameter(override.get_name(), override.get_parameter_value());
    }
}

ManipulatorRosNode::~ManipulatorRosNode() 
{
    RCLCPP_INFO(get_logger(), "Manipulator Node is destroyed");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ManipulatorRosNode::on_configure(const rclcpp_lifecycle::State&) 
{
    RCLCPP_INFO(get_logger(), "Manipulator Node configured");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ManipulatorRosNode::on_activate(const rclcpp_lifecycle::State& state) 
{
    RCLCPP_INFO(get_logger(), "Manipulator Node is activated");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ManipulatorRosNode::on_deactivate(const rclcpp_lifecycle::State& state)
{
    RCLCPP_INFO(get_logger(), "Manipulator Node is de-activated");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ManipulatorRosNode::on_cleanup(const rclcpp_lifecycle::State&)
{
    RCLCPP_INFO(get_logger(), "Manipulator Node cleaned up");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ManipulatorRosNode::on_shutdown(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(get_logger(), "Manipulator Node shutdown");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ManipulatorRosNode>(rclcpp::NodeOptions());
    auto ethercat_path = ament_index_cpp::get_package_share_directory("youbot_driver");
    string file_path = ethercat_path + "/config";
    manipulation_namespace::Manipulator manipulator(file_path);
    RCLCPP_INFO(node->get_logger(), "Your ROS wrapper node is working.");
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}