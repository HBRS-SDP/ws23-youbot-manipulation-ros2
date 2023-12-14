/*
* Copyright 2023 Bonn-Rhein-Sieg University
*
* ROS2 authors: Jay Parikh, Kamendu Panchal, Chaitanya Gumudala, Mohsen Azizmalayeri.
*
*/

#include "mir_youbot_manipulation/youbot_manipulation_node.hpp"

KDL::Tree youbot_tree;
KDL::Chain youbot_kdl_chain;

ManipulatorRosNode::ManipulatorRosNode(const rclcpp::NodeOptions& options) : rclcpp_lifecycle::LifecycleNode("youbot_manipulation_node", options)
{
    RCLCPP_INFO(get_logger(), "Manipulator Node is created");
    this->declare_parameter("robot_description", " ");
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
    // youbot_manipulator = std::make_shared<manipulation_namespace::Manipulator>(ethercat_path);
    youbot_manipulator = std::make_shared<manipulation_namespace::Manipulator>();
    
    
    std::string robot_description = this->get_parameter("robot_description").as_string();
    if (!kdl_parser::treeFromString(robot_description, youbot_tree))
    {
        RCLCPP_INFO(get_logger(), "Unable to get parameters");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    if (!youbot_tree.getChain("arm_link_0", "arm_link_5", youbot_kdl_chain))
    {
        RCLCPP_INFO(get_logger(), "Unable to get chain");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    // KDL::JntArray joint_angles(youbot_kdl_chain.getNrOfJoints());
    // // joint_angles.data.setZero();
    // joint_angles.data[0] = 0.174533;
    // joint_angles.data[1] = 0.174533;
    // joint_angles.data[2] = 0.174533;
    // joint_angles.data[3] = 0.174533;
    // joint_angles.data[4] = 0.174533;

    // std::vector<JointAngleSetpoint> joint_positions;
    // for (int i = 0; i <= joint_angles.rows(); i ++) {
    //     joint_positions[i].angle = joint_angles.data[i] * radian;
    // }
    // youbot_manipulator -> moveArmJoints(joint_positions);
    // KDL::Frame target_pose;
    // youbot_manipulator -> forwardKinematics(joint_angles, youbot_kdl_chain, target_pose);
    // youbot_manipulator -> inverseKinematics(target_pose, youbot_kdl_chain, joint_angles);
    
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
        if (joint_position.unit == "rad") {
            value = value * 180.0/ M_PI ;
        }
        joint_positions.push_back(value);

    }
    KDL::Frame target_pose;
    KDL::JntArray joint_angles(youbot_kdl_chain.getNrOfJoints());
    for (int i = 0; i < joint_angles.rows(); i ++) {
        joint_angles(i) = joint_positions[i] * M_PI/180;
    }
    auto youbot_angles_set_point = youbot_manipulator -> convertDoubleToJointAngleSetpoint(joint_positions);
    youbot_manipulator -> forwardKinematics(joint_angles, youbot_kdl_chain, target_pose);
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
    KDL::Frame target_pose;
    const auto& cartesian_coordinates = goal->cartesian_coordinates;
    target_pose.p.x(cartesian_coordinates.pose.position.x);
    target_pose.p.y(cartesian_coordinates.pose.position.y);
    target_pose.p.z(cartesian_coordinates.pose.position.z);
    target_pose.M = KDL::Rotation::Quaternion(
        cartesian_coordinates.pose.orientation.x,
        cartesian_coordinates.pose.orientation.y,
        cartesian_coordinates.pose.orientation.z,
        cartesian_coordinates.pose.orientation.w
    );

    KDL::JntArray joint_angles(youbot_kdl_chain.getNrOfJoints());
    youbot_manipulator -> inverseKinematics(target_pose, youbot_kdl_chain, joint_angles);
    std::vector<JointAngleSetpoint> joint_angles_setpoint;
    for (int i = 0; i < joint_angles.rows(); i++) 
    {
        JointAngleSetpoint joint_angle_setpoint;
        joint_angle_setpoint.angle = (joint_angles(i) * 180.0 / M_PI) * radian;
        joint_angles_setpoint.push_back(joint_angle_setpoint);
    }

    youbot_manipulator -> moveArmJoints(joint_angles_setpoint);
    auto result = std::make_shared<mir_interfaces::action::CartesianCoordinates::Result>();
    goal_handle->succeed(result);
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ManipulatorRosNode>(rclcpp::NodeOptions());
    
    // RCLCPP_INFO(node->get_logger(), "Your ROS wrapper node is working.");
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}