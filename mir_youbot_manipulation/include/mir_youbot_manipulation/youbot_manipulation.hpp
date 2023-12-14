/*
* Copyright 2023 Bonn-Rhein-Sieg University
*
* ROS2 authors: Jay Parikh, Kamendu Panchal, Chaitanya Gumudala, Mohsen Azizmalayeri.
*
*/
 
#include <iostream> 
#include <vector>
#include <cmath>
#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "youbot_driver/youbot/EthercatMasterWithThread.hpp"
#include "youbot_driver/youbot/YouBotManipulator.hpp"
#include "youbot_driver/youbot/DataTrace.hpp"
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include "kdl/chainiksolverpos_lma.hpp"
#include <kdl/tree.hpp>

using namespace youbot;

namespace manipulation_namespace{
	class Manipulator{
		public:			
			// Manipulator(const std::string &file_path);
			Manipulator();
			/**
			  * \brief Moves youBot arm according to the input angles 
			  * \param[in] joint_angles_deg in double as a list of angles in degrees
			  * \param[out] bool indicating the movement is done or not
			  */
			bool moveArmJoints(const std::vector<JointAngleSetpoint> &joint_angles_deg);

			/**
			  * \brief Converts vector double values to joint angle set points data type
			  * \param[in] vector<double> as a list of angles in youbot convention
			  * \param[out] vector<JointAngleSetpoint>
			  */
    		vector<JointAngleSetpoint> convertDoubleToJointAngleSetpoint(const std::vector<double> &input_angle);

			/**
			  * \brief Perform inverse kinematic
			  * \param[in] target_pose position to move arm joints
			  * \param[in] chain a chain from arm_link_0 to arm_link_5
			  * \param[out] joint_angles a list of joint angles to be executed on youBot arm
			  */
			bool inverseKinematics(const KDL::Frame& target_pose, const KDL::Chain& chain, KDL::JntArray& joint_angles_return);

			/**
			  * \brief Perform forward kinematic
			  * \param[in] joint_angles a list of joint angles to be executed on youBot arm
			  * \param[in] chain a chain from arm_link_0 to arm_link_5
			  * \param[out] target_pose position to move arm joints
			  */
			bool forwardKinematics(const KDL::JntArray& joint_angles, const KDL::Chain& chain, KDL::Frame& target_pose);
			
		private:
			vector<JointAngleSetpoint> minimum_angles;
    		vector<JointAngleSetpoint> maximum_angles;
			vector<JointAngleSetpoint> compensate_angles;
			// YouBotManipulator myArm;
			std::string ethercat_path;


			void readYAML();

			/**
			  * \brief Converts degrees to radians
			  * \param[in] vector<double> as a list of angles in degree
			  * \param[out] vector<double> 
			  */
    		void convertDegToRad(const std::vector<JointAngleSetpoint> &joint_angles_deg, std::vector<JointAngleSetpoint> &joint_angles_rad);
			
			/**
			  * \brief Checks whether input angles are in the range of youBot's physical configuration or not
			  * \param[in] vector<double> as a list of angles in radian
			  * \param[out] true or false
			  */
    		bool validateInput(const std::vector<JointAngleSetpoint> &joint_angles_rad);
			
			/**
			  * \brief Adds compensation angles to the input angles
			  * \param[in] vector<double> as a list of angles in radian
			  * \param[in] vector<double> as a list of angles to compensate in radian
			  * \param[out] vector<JointAngleSetpoint>
			  */
    		void convertJointAnglesToYoubotDriverConvention(const std::vector<JointAngleSetpoint> &joint_angles_rad, const std::vector<JointAngleSetpoint> &compensate_angles, std::vector<JointAngleSetpoint> &youbot_angles_set_point);
	};
}
