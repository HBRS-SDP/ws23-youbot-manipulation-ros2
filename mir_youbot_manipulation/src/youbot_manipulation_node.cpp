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
    auto ethercat_path = ament_index_cpp::get_package_share_directory("youbot_driver") + "/config";
    // manipulation_namespace::Manipulator manipulator(ethercat_path);
    youbot_manipulator = std::make_shared<manipulation_namespace::Manipulator>(ethercat_path);
    KDL::Tree youbot_tree;
    auto youbot_urdf_path = ament_index_cpp::get_package_share_directory("youbot_description") + "/urdf/youbot_arm/arm_corrected_dynamics.urdf.xacro";
    if (!kdl_parser::treeFromFile(youbot_urdf_path, youbot_tree)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF file");
    }
    KDL::Chain youbot_kdl_chain;
    if (!youbot_tree.getChain("arm_link_0", "arm_link_5", youbot_kdl_chain)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create KDL chain");
    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ManipulatorRosNode::on_activate(const rclcpp_lifecycle::State& state) 
{
    RCLCPP_INFO(get_logger(), "Manipulator Node is activated");
    joint_positions_action_server = rclcpp_action::create_server<mir_interfaces::action::MoveToJointAngles>(
      shared_from_this(),
      "~/joint_angles",
      std::bind(
        &ManipulatorRosNode::manipulatorHandleCallback, this, std::placeholders::_1,
        std::placeholders::_2),
      std::bind(
        &ManipulatorRosNode::manipulatorSelectorCancelCallback, this,
        std::placeholders::_1),
      std::bind(
        &ManipulatorRosNode::manipulatorAcceptedCallback, this,
        std::placeholders::_1));

    cartesian_pose_action_server = rclcpp_action::create_server<mir_interfaces::action::CartesianCoordinates>(
      shared_from_this(),
      "~/cartesian_pose",
      std::bind(
        &ManipulatorRosNode::cartesianPoseHandleCallback, this, std::placeholders::_1,
        std::placeholders::_2),
      std::bind(
        &ManipulatorRosNode::cartesianPoseSelectorCancelCallback, this,
        std::placeholders::_1),
      std::bind(
        &ManipulatorRosNode::cartesianPoseAcceptedCallback, this,
        std::placeholders::_1));

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ManipulatorRosNode::on_deactivate(const rclcpp_lifecycle::State& state)
{
    RCLCPP_INFO(get_logger(), "Manipulator Node is de-activated");
    LifecycleNode::on_deactivate(state);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ManipulatorRosNode::on_cleanup(const rclcpp_lifecycle::State&)
{
    RCLCPP_INFO(get_logger(), "Manipulator Node cleaned up");
    joint_positions_action_server.reset();
    cartesian_pose_action_server.reset();
    youbot_manipulator.reset();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ManipulatorRosNode::on_shutdown(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(get_logger(), "Manipulator Node shutdown");
    LifecycleNode::on_shutdown(state);
    delete joint_positions_action_server.get();
    delete cartesian_pose_action_server.get();
    delete youbot_manipulator.get();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


rclcpp_action::GoalResponse 
ManipulatorRosNode::manipulatorHandleCallback(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const mir_interfaces::action::MoveToJointAngles::Goal> goal)
{
    RCLCPP_INFO(get_logger(), "Manipulator Node received joint positions goal");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse 
ManipulatorRosNode:: manipulatorSelectorCancelCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<mir_interfaces::action::MoveToJointAngles>> goal_handle)
{
    RCLCPP_INFO(get_logger(), "Manipulator Node received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ManipulatorRosNode::manipulatorAcceptedCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<mir_interfaces::action::MoveToJointAngles>> goal_handle)
{
    RCLCPP_INFO(get_logger(), "Manipulator Node accepted object selector goal");
    std::thread{std::bind(&ManipulatorRosNode::executeManipulator, this, std::placeholders::_1), goal_handle}.detach();
}

void ManipulatorRosNode::executeManipulator(const std::shared_ptr<rclcpp_action::ServerGoalHandle<mir_interfaces::action::MoveToJointAngles>> goal_handle)
{
    RCLCPP_INFO(get_logger(), "Manipulator Node executing object selector goal");
    // ManipulatorRosNode::on_configure();
    const auto goal = goal_handle -> get_goal();
    mir_interfaces::action::MoveToJointAngles::Goal moveArmJointsGoal;
    const auto& joint_positions_setpoint = goal->joint_positions_setpoint;
    std::vector<double> joint_positions;
    for (const auto& joint_position : joint_positions_setpoint.positions) {
        double value = joint_position.value;
        if (joint_position.unit == "deg") {
            value = value * M_PI / 180.0;
        }
        joint_positions.push_back(value);

    }
    auto youbot_angles_set_point = youbot_manipulator -> convertDoubleToJointAngleSetpoint(joint_positions);
    youbot_manipulator -> moveArmJoints(youbot_angles_set_point);
    auto result = std::make_shared<mir_interfaces::action::MoveToJointAngles::Result>();
    goal_handle->succeed(result);

}

rclcpp_action::GoalResponse 
ManipulatorRosNode::cartesianPoseHandleCallback(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const mir_interfaces::action::CartesianCoordinates::Goal> goal)
{
    RCLCPP_INFO(get_logger(), "Manipulator Node received cartesian positions goal");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse 
ManipulatorRosNode::cartesianPoseSelectorCancelCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<mir_interfaces::action::CartesianCoordinates>> goal_handle)
{
    RCLCPP_INFO(get_logger(), "Manipulator Node received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ManipulatorRosNode::cartesianPoseAcceptedCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<mir_interfaces::action::CartesianCoordinates>> goal_handle)
{
    RCLCPP_INFO(get_logger(), "Manipulator Node accepted object selector goal");
    std::thread{std::bind(&ManipulatorRosNode::executeCartesianPose, this, std::placeholders::_1), goal_handle}.detach();
}

void ManipulatorRosNode::executeCartesianPose(const std::shared_ptr<rclcpp_action::ServerGoalHandle<mir_interfaces::action::CartesianCoordinates>> goal_handle)
{
    RCLCPP_INFO(get_logger(), "Manipulator Node executing object selector goal");
    const auto goal = goal_handle -> get_goal();
    mir_interfaces::action::CartesianCoordinates::Goal moveArmPoseGoal;
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ManipulatorRosNode>(rclcpp::NodeOptions());
    
    // RCLCPP_INFO(node->get_logger(), "Your ROS wrapper node is working.");
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}