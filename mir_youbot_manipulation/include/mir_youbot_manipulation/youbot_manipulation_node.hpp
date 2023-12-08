/*
* Copyright 2023 Bonn-Rhein-Sieg University
*
* ROS2 authors: Jay Parikh, Kamendu Panchal, Chaitanya Gumudala, Mohsen Azizmalayeri.
*
*/


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "brics_actuator/msg/joint_positions.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "mir_interfaces/action/move_to_joint_angles.hpp"
#include "mir_interfaces/action/cartesian_coordinates.hpp"
#include "mir_youbot_manipulation/youbot_manipulation.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <kdl_parser/kdl_parser.hpp>


class ManipulatorRosNode: public rclcpp_lifecycle::LifecycleNode
{ 
	public: 
 		explicit ManipulatorRosNode(const rclcpp::NodeOptions& options); 

		virtual ~ManipulatorRosNode();
  
 		/// Transition callback for state configuring 
 		rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State&); 
  
 		/// Transition callback for state activating 
 		rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State& state); 
  
 		/// Transition callback for state deactivating 
 		rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state); 
 		  
		/// Transition callback for state cleaningup 
 		rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State&); 
  
 		/// Transition callback for state shutting down 
 		rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state);
		
	private:
		// ============================ Members ============================
		std::shared_ptr<manipulation_namespace::Manipulator> youbot_manipulator;
	
		// manipulator action server
		rclcpp_action::Server<mir_interfaces::action::MoveToJointAngles>::SharedPtr joint_positions_action_server;

		//cartesian pose action server
		rclcpp_action::Server<mir_interfaces::action::CartesianCoordinates>::SharedPtr cartesian_pose_action_server;

		// action server callbacks
		rclcpp_action::GoalResponse manipulatorHandleCallback(
			const rclcpp_action::GoalUUID& uuid,
			std::shared_ptr<const mir_interfaces::action::MoveToJointAngles::Goal> goal);

		rclcpp_action::CancelResponse manipulatorSelectorCancelCallback(
			const std::shared_ptr<rclcpp_action::ServerGoalHandle<
			mir_interfaces::action::MoveToJointAngles>> goal_handle);

		void manipulatorAcceptedCallback(
			const std::shared_ptr<rclcpp_action::ServerGoalHandle<
			mir_interfaces::action::MoveToJointAngles>> goal_handle);

		rclcpp_action::GoalResponse cartesianPoseHandleCallback(
			const rclcpp_action::GoalUUID& uuid,
			std::shared_ptr<const mir_interfaces::action::CartesianCoordinates::Goal> goal);

		rclcpp_action::CancelResponse cartesianPoseSelectorCancelCallback(
			const std::shared_ptr<rclcpp_action::ServerGoalHandle<
			mir_interfaces::action::CartesianCoordinates>> goal_handle);

		void cartesianPoseAcceptedCallback(
			const std::shared_ptr<rclcpp_action::ServerGoalHandle<
			mir_interfaces::action::CartesianCoordinates>> goal_handle);


		// ============================ Methods ============================
		void executeManipulator(
			const std::shared_ptr<rclcpp_action::ServerGoalHandle<
			mir_interfaces::action::MoveToJointAngles>> goal_handle);

		void executeCartesianPose(
			const std::shared_ptr<rclcpp_action::ServerGoalHandle<
			mir_interfaces::action::CartesianCoordinates>> goal_handle);
};