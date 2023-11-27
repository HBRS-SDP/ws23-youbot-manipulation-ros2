/*
* Copyright 2023 Bonn-Rhein-Sieg University
*
* ROS2 authors: Jay Parikh, Kamendu Panchal, Chaitanya Gumudala, Mohsen Azizmalayeri.
*
*/


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "brics_actuator/msg/joint_positions.hpp"
#include "mir_youbot_manipulation/youbot_manipulation.hpp"


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
};