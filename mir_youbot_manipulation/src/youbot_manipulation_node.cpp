/*
 * Copyright 2023 Bonn-Rhein-Sieg University
 *
 * ROS2 authors: Jay Parikh, Kamendu Panchal, Chaitanya Gumudala, Mohsen Azizmalayeri.
 *
 */

#include "mir_youbot_manipulation/youbot_manipulation_node.hpp"

ManipulatorRosNode::ManipulatorRosNode(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("youbot_manipulation_node", options)
{
  RCLCPP_INFO(get_logger(), "Manipulator Node is created");
  this->declare_parameter("robot_description", " ");
}

ManipulatorRosNode::~ManipulatorRosNode()
{
  RCLCPP_INFO(get_logger(), "Manipulator Node is destroyed");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ManipulatorRosNode::on_configure(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(get_logger(), "Manipulator Node configured");
  auto ethercat_path =
      ament_index_cpp::get_package_share_directory("youbot_driver") + "/config";
  // manipulation_namespace::Manipulator manipulator(ethercat_path);
  youbot_manipulator =
      std::make_shared<manipulation_namespace::Manipulator>(ethercat_path);
  // youbot_manipulator = std::make_shared<manipulation_namespace::Manipulator>();
  std::string robot_description = this->get_parameter("robot_description").as_string();
  if (!kdl_parser::treeFromString(robot_description, youbot_tree))
  {
    RCLCPP_INFO(get_logger(), "Unable to get parameters");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::
        FAILURE;
  }
  if (!youbot_tree.getChain("arm_link_0", "arm_link_5", youbot_kdl_chain))
  {
    RCLCPP_INFO(get_logger(), "Unable to get chain");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::
        FAILURE;
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::
      SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ManipulatorRosNode::on_activate(const rclcpp_lifecycle::State& state)
{
  RCLCPP_INFO(get_logger(), "Manipulator Node is activated");
  move_to_joint_angles =
      rclcpp_action::create_server<mir_interfaces::action::MoveToJointAngles>(
          shared_from_this(), "~/joint_angles",
          std::bind(&ManipulatorRosNode::jointAnglesHandleCallback, this,
                    std::placeholders::_1, std::placeholders::_2),
          std::bind(&ManipulatorRosNode::jointAnglesCancelCallback, this,
                    std::placeholders::_1),
          std::bind(&ManipulatorRosNode::jointAnglesAcceptedCallback, this,
                    std::placeholders::_1));

  move_to_cartesian_pose =
      rclcpp_action::create_server<mir_interfaces::action::MoveToCartesianPose>(
          shared_from_this(), "~/cartesian_pose",
          std::bind(&ManipulatorRosNode::cartesianPoseHandleCallback, this,
                    std::placeholders::_1, std::placeholders::_2),
          std::bind(&ManipulatorRosNode::cartesianPoseCancelCallback, this,
                    std::placeholders::_1),
          std::bind(&ManipulatorRosNode::cartesianPoseAcceptedCallback, this,
                    std::placeholders::_1));

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::
      SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ManipulatorRosNode::on_deactivate(const rclcpp_lifecycle::State& state)
{
  RCLCPP_INFO(get_logger(), "Manipulator Node is de-activated");
  LifecycleNode::on_deactivate(state);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::
      SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ManipulatorRosNode::on_cleanup(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(get_logger(), "Manipulator Node cleaned up");
  move_to_joint_angles.reset();
  move_to_cartesian_pose.reset();
  youbot_manipulator.reset();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::
      SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ManipulatorRosNode::on_shutdown(const rclcpp_lifecycle::State& state)
{
  RCLCPP_INFO(get_logger(), "Manipulator Node shutdown");
  LifecycleNode::on_shutdown(state);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::
      SUCCESS;
}

rclcpp_action::GoalResponse ManipulatorRosNode::jointAnglesHandleCallback(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const mir_interfaces::action::MoveToJointAngles::Goal> goal)
{
  RCLCPP_INFO(get_logger(), "Manipulator Node received joint positions goal");
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ManipulatorRosNode::jointAnglesCancelCallback(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<mir_interfaces::action::MoveToJointAngles>>
        goal_handle)
{
  RCLCPP_INFO(get_logger(), "Manipulator Node received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ManipulatorRosNode::jointAnglesAcceptedCallback(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<mir_interfaces::action::MoveToJointAngles>>
        goal_handle)
{
  RCLCPP_INFO(get_logger(), "Manipulator Node accepted object selector goal");
  std::thread{
      std::bind(&ManipulatorRosNode::executeJointAngles, this, std::placeholders::_1),
      goal_handle}
      .detach();
}

void ManipulatorRosNode::executeJointAngles(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<mir_interfaces::action::MoveToJointAngles>>
        goal_handle)
{
  RCLCPP_INFO(get_logger(), "Manipulator Node executing object selector goal");
  const auto goal = goal_handle->get_goal();
  mir_interfaces::action::MoveToJointAngles::Goal moveArmJointsGoal;
  const auto& joint_positions = goal->joint_positions;

  for (const auto& joint_position : joint_positions.positions)
  {
    const auto& joint_uri = joint_position.joint_uri;
    if (std::find(expected_joint_names.begin(), expected_joint_names.end(), joint_uri) ==
        expected_joint_names.end())
    {
      RCLCPP_ERROR(get_logger(), "Invalid joint URI: %s", joint_uri.c_str());
      auto result = std::make_shared<mir_interfaces::action::MoveToJointAngles::Result>();
      goal_handle->abort(result);
      return;
    }
  }

  vector<JointAngleSetpoint> joint_angle_setpoints;
  for (const auto& joint_position : joint_positions.positions)
  {
    JointAngleSetpoint value;
    double input_angle = joint_position.value;
    if (joint_position.unit == "deg")
    {
      input_angle = input_angle * M_PI / 180;
    }
    value.angle = (input_angle)*radian;
    joint_angle_setpoints.push_back(value);
  }

  youbot_manipulator->moveArmJoints(joint_angle_setpoints);
  auto result = std::make_shared<mir_interfaces::action::MoveToJointAngles::Result>();
  goal_handle->succeed(result);
}

rclcpp_action::GoalResponse ManipulatorRosNode::cartesianPoseHandleCallback(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const mir_interfaces::action::MoveToCartesianPose::Goal> goal)
{
  RCLCPP_INFO(get_logger(), "Manipulator Node received cartesian positions goal");
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ManipulatorRosNode::cartesianPoseCancelCallback(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<mir_interfaces::action::MoveToCartesianPose>>
        goal_handle)
{
  RCLCPP_INFO(get_logger(), "Manipulator Node received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ManipulatorRosNode::cartesianPoseAcceptedCallback(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<mir_interfaces::action::MoveToCartesianPose>>
        goal_handle)
{
  RCLCPP_INFO(get_logger(), "Manipulator Node accepted object selector goal");
  std::thread{
      std::bind(&ManipulatorRosNode::executeCartesianPose, this, std::placeholders::_1),
      goal_handle}
      .detach();
}

void ManipulatorRosNode::executeCartesianPose(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<mir_interfaces::action::MoveToCartesianPose>>
        goal_handle)
{
  RCLCPP_INFO(get_logger(), "Manipulator Node executing object selector goal");
  const auto goal = goal_handle->get_goal();
  mir_interfaces::action::MoveToCartesianPose::Goal moveArmPoseGoal;
  const auto& cartesian_coordinates = goal->cartesian_coordinates;
  KDL::Frame target_pose;
  tf2::fromMsg(cartesian_coordinates.pose, target_pose);

  KDL::JntArray joint_angles(youbot_kdl_chain.getNrOfJoints());
  youbot_manipulator->inverseKinematics(target_pose, youbot_kdl_chain, joint_angles);
  std::vector<JointAngleSetpoint> joint_angles_setpoint;

  for (int i = 0; i < joint_angles.rows(); i++)
  {
    JointAngleSetpoint joint_angle_setpoint;
    joint_angle_setpoint.angle = joint_angles(i) * radian;
    joint_angles_setpoint.push_back(joint_angle_setpoint);
  }

  youbot_manipulator->moveArmJoints(joint_angles_setpoint);
  auto result = std::make_shared<mir_interfaces::action::MoveToCartesianPose::Result>();
  goal_handle->succeed(result);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ManipulatorRosNode>(rclcpp::NodeOptions());
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}