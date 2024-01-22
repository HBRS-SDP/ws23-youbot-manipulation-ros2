/*
 * Copyright 2023 Bonn-Rhein-Sieg University
 *
 * ROS2 authors: Jay Parikh, Kamendu Panchal, Chaitanya Gumudala, Mohsen Azizmalayeri.
 *
 */

#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <tf2_kdl/tf2_kdl.hpp>

#include "brics_actuator/msg/joint_positions.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mir_interfaces/action/move_to_cartesian_pose.hpp"
#include "mir_interfaces/action/move_to_joint_angles.hpp"
#include "mir_interfaces/action/move_using_joint_velocities.hpp"
#include "mir_youbot_manipulation/youbot_manipulation.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

class ManipulatorRosNode : public rclcpp_lifecycle::LifecycleNode
{
  const std::vector<std::string> expected_joint_names = {
      "arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"};

public:
  explicit ManipulatorRosNode(const rclcpp::NodeOptions& options);

  virtual ~ManipulatorRosNode();

  /// Transition callback for state configuring
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State&);

  /// Transition callback for state activating
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& state);

  /// Transition callback for state deactivating
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& state);

  /// Transition callback for state cleaningup
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State&);

  /// Transition callback for state shutting down
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
      const rclcpp_lifecycle::State& state);

private:
  KDL::Tree youbot_tree;
  KDL::Chain youbot_kdl_chain;
  // ============================ Members ============================
  std::shared_ptr<manipulation_namespace::Manipulator> youbot_manipulator;

  // manipulator action server
  rclcpp_action::Server<mir_interfaces::action::MoveToJointAngles>::SharedPtr
      move_to_joint_angles;

  // cartesian pose action server
  rclcpp_action::Server<mir_interfaces::action::MoveToCartesianPose>::SharedPtr
      move_to_cartesian_pose;

  rclcpp_action::Server<mir_interfaces::action::MoveUsingJointVelocities>::SharedPtr
      move_using_joint_velocities;

  // action server callbacks
  rclcpp_action::GoalResponse jointAnglesHandleCallback(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const mir_interfaces::action::MoveToJointAngles::Goal> goal);

  rclcpp_action::CancelResponse jointAnglesCancelCallback(
      const std::shared_ptr<
          rclcpp_action::ServerGoalHandle<mir_interfaces::action::MoveToJointAngles>>
          goal_handle);

  void jointAnglesAcceptedCallback(
      const std::shared_ptr<
          rclcpp_action::ServerGoalHandle<mir_interfaces::action::MoveToJointAngles>>
          goal_handle);

  rclcpp_action::GoalResponse cartesianPoseHandleCallback(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const mir_interfaces::action::MoveToCartesianPose::Goal> goal);

  rclcpp_action::CancelResponse cartesianPoseCancelCallback(
      const std::shared_ptr<
          rclcpp_action::ServerGoalHandle<mir_interfaces::action::MoveToCartesianPose>>
          goal_handle);

  void cartesianPoseAcceptedCallback(
      const std::shared_ptr<
          rclcpp_action::ServerGoalHandle<mir_interfaces::action::MoveToCartesianPose>>
          goal_handle);

  rclcpp_action::GoalResponse jointVelocitiesHandleCallback(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const mir_interfaces::action::MoveUsingJointVelocities::Goal> goal);

  rclcpp_action::CancelResponse jointVelocitiesCancelCallback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<
          mir_interfaces::action::MoveUsingJointVelocities>>
          goal_handle);

  void jointVelocitiesAcceptedCallback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<
          mir_interfaces::action::MoveUsingJointVelocities>>
          goal_handle);

  // ============================ Methods ============================
  void executeJointAngles(
      const std::shared_ptr<
          rclcpp_action::ServerGoalHandle<mir_interfaces::action::MoveToJointAngles>>
          goal_handle);

  void executeCartesianPose(
      const std::shared_ptr<
          rclcpp_action::ServerGoalHandle<mir_interfaces::action::MoveToCartesianPose>>
          goal_handle);
  void executeJointVelocities(const std::shared_ptr<rclcpp_action::ServerGoalHandle<
                                  mir_interfaces::action::MoveUsingJointVelocities>>
                                  goal_handle);
};