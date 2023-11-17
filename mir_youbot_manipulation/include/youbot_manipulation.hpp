/*
 * Copyright 2023 Bonn-Rhein-Sieg University
 *
 * ROS2 contributors: Jay Parikh, Kamendu Panchal, Chaitanya Gumudala, Mohsen Azizmalayeri.
 *
 */
 
#include <iostream> 
#include <vector>
#include <cmath>
#include <yaml-cpp/yaml.h>

#include "youbot_driver/youbot/EthercatMasterWithThread.hpp"
#include "youbot_driver/youbot/YouBotManipulator.hpp"
#include "youbot_driver/youbot/DataTrace.hpp"
 
using namespace std;
using namespace youbot;

namespace manipulation_namespace{
	class MultiModalObjectRecognitionROS: public rclcpp_lifecycle::LifecycleNode{
		public:
			explicit MultiModalObjectRecognitionROS(const rclcpp::NodeOptions& options);

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

		// --------------------------- function declarations -----------------------------------
		
			/**
			  * \brief Moves youBot arm according to the input angles 
			  * \param[in] joint_angles in double as a list of angles in degrees
			  * \param[out] true or false indicating the movement is done or not
			  */
			bool moveJointArms(vector<double> input_angles);
			
		private:
		
			/**
			  * \brief Converts vector double values to joint angle set points data type
			  * \param[in] input_angles in vector<double> as a list of angles
			  * \param[out] vector<JointAngleSetpoint>
			  */
			vector<JointAngleSetpoint> convertDoubleToJointAngleSetpoint(vector<double> input_angles);
			
			/**
			  * \brief Converts degrees to radians
			  * \param[in] degrees in vector<double> as a list of angles
			  * \param[out] rad_angles as vector<double> 
			  */
			vector<double> convertDegToRad(vector<double> deg_angles);
			
			/**
			  * \brief Checks whether input angles are in the range of youBot's physical configuration or not
			  * \param[in] joint_angles in vector<double> as a list of angles
			  * \param[out] true or false
			  */
			bool validateInput(vector<double> input_angles);

			/**
			  * \brief Subtracts compensation angles from the input_angles
			  * \param[in] input_angles in vector<double> as a list of angles
			  * \param[in] compensate_angles in vector<double> as a list of angles
			  * \param[out] vector<JointAngleSetpoint>
			  */
			vector<JointAngleSetpoint> convertJointAnglesToJointConvention(vector<JointAngleSetpoint> input_angles, vector<JointAngleSetpoint> vector<double> compensate_angles);
			
			/**
			  * \brief Adds compensation angles to the input_angles
			  * \param[in] input_angles in vector<double> as a list of angles
			  * \param[in] compensate_angles in vector<double> as a list of angles
			  * \param[out] output_angles as vector<JointAngleSetpoint>
			  */
			vector<JointAngleSetpoint> convertJointAnglesToYouBotConvention(vector<double> input_angles, vector<double> compensate_angles);
	}
}
