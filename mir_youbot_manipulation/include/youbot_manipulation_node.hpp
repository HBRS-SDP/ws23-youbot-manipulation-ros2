class ManipulatorROS: public rclcpp_lifecycle::LifecycleNode{ 
	public: 
 		explicit ManipulatorROS(const rclcpp::NodeOptions& options); 
  
 		/// Transition callback for state configuring 
 		rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
 		on_configure(const rclcpp_lifecycle::State &); 
  
 		/// Transition callback for state activating 
 		rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
 		on_activate(const rclcpp_lifecycle::State & state); 
  
 		/// Transition callback for state deactivating 
 		rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
 		on_deactivate(const rclcpp_lifecycle::State & state); 
  
 		  /// Transition callback for state cleaningup 
 		rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
 		on_cleanup(const rclcpp_lifecycle::State &); 
  
 		/// Transition callback for state shutting down 
 		rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
 		on_shutdown(const rclcpp_lifecycle::State & state);
};
