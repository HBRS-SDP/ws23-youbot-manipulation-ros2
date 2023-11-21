/*
 * Copyright 2023 Bonn-Rhein-Sieg University
 *
 * ROS2 authors: Jay Parikh, Kamendu Panchal, Chaitanya Gumudala, Mohsen Azizmalayeri.
 *
 */

#include "../include/youbot_manipulation.hpp"


using namespace youbot;
//using namespace manipulation_namespace

class Manipulator{
    private:
        vector<double> minimum_angles;
        vector<double> maximum_angles;
        vector<double> compensate_angles;
        // YouBotManipulator myArm;

        void readYAML() {
            auto ros2_path = ament_index_cpp::get_package_share_directory("mir_youbot_manipulation");
            string file_path = ros2_path + "/config/joint_limits.yaml";
            YAML::Node node = YAML::LoadFile(file_path);
            if (node["mir_youbot_manipulation"]) {
                YAML::Node youbot_manipulation = node["mir_youbot_manipulation"];
                YAML::Node joint_angles = youbot_manipulation["joint_angles"];
                for (const auto& joint : joint_angles) {
                    double minimum = joint["min"].as<double>();
                    double maximum = joint["max"].as<double>();
                    double compensate = joint["compensate"].as<double>();
                    minimum_angles.push_back(minimum);
                    maximum_angles.push_back(maximum);
                    compensate_angles.push_back(compensate);
                }
            }
        }
        
        vector<double> convertDegToRad(vector<double> joint_angles_deg) {
            vector<double> joint_angles_rad;
			for (int i = 0; i < joint_angles_deg.size(); i++) {
                joint_angles_rad.push_back(joint_angles_deg[i] * M_PI / 180.0);
            }
			return joint_angles_rad;
        }

        bool validateInput(vector<double> joint_angles_rad) {
            if (joint_angles_rad.size() != minimum_angles.size() || joint_angles_rad.size() != maximum_angles.size()) {
                std::cout << "Error: Input vector size does not match expected size" << endl;
                return false;    
            }
            for (int i = 0; i < joint_angles_rad.size(); i++) {
                if (joint_angles_rad[i] < minimum_angles[i] || joint_angles_rad[i] > maximum_angles[i]) {
                    std::cout << "Joint angle " << i + 1 << " is out of range" << std::endl;
                    return false;
                }
            }
            return true;
        }

        vector<double> convertJointAnglesToYoubotDriverConvention(vector<double> joint_angles_rad, vector<double> compensate_angles) {
            vector<double> youbot_driver_joint_angles;
			youbot_driver_joint_angles.resize(joint_angles_rad.size());
            for(int i = 0; i < joint_angles_rad.size(); i++) {
                youbot_driver_joint_angles[i] = compensate_angles[i] + joint_angles_rad[i];
            }
			return youbot_driver_joint_angles;
        }

        vector<JointAngleSetpoint> convertDoubleToJointAngleSetpoint(vector<double> youbot_driver_joint_angles) {
            vector<JointAngleSetpoint> youbot_angles_set_point;
			for (int i = 0; i < youbot_driver_joint_angles.size(); i++) {
                JointAngleSetpoint youbot_angle_set_point;
                youbot_angle_set_point.angle = youbot_driver_joint_angles[i] * radian;
                youbot_angles_set_point.push_back(youbot_angle_set_point);
            }
			return youbot_angles_set_point;
        }

    public:
        Manipulator(){ 
            // :myArm("youbot-manipulator", "/home/chaitanya/workspace_sdp/src/youbot_driver/config") {
            readYAML();
			// myArm.doJointCommutation();
            // myArm.calibrateManipulator();
        }
        
        bool moveArmJoints(vector<double> joint_angles_deg){
			vector<double> joint_angles_rad = convertDegToRad(joint_angles_deg);
            if(validateInput(joint_angles_rad)){
				vector<double> youbot_driver_joint_angles = convertJointAnglesToYoubotDriverConvention(joint_angles_rad, compensate_angles);
                vector<JointAngleSetpoint> youbot_angles_set_point = convertDoubleToJointAngleSetpoint(youbot_driver_joint_angles);
                for (int i = 0; i < youbot_angles_set_point.size(); i++) {
                    std::cout << "Input joint " << i + 1 << " angle: " << youbot_angles_set_point[i].angle.value() <<  std::endl;
                }
                // myArm.setJointData(youbot_angles_set_point);
                // sleep(5);
                vector<JointSensedAngle> youbot_sensed_angles;
				// myArm.getJointData(youbot_sensed_angles);
                for (int i = 0; i < youbot_angles_set_point.size() ; i++) {
                    std::cout << "Current joint " << i  << " angle: " << youbot_sensed_angles[i].angle.value() <<  std::endl;
                }
                return true;
            } else {
                return false;
            }
        }
};

int main(int argc, char **argv) {
    // EthercatMaster::getInstance("youbot-ethercat.cfg", "/home/chaitanya/workspace_sdp/src/youbot_driver/config", true);
	vector<double> input_angles;
    for (int i = 0; i < 5; i++) {
        double angle;
        std::cout << "Enter joint " << i + 1 << " angle : ";
        std::cin >> angle;
        input_angles.push_back(angle);
    }
    
    Manipulator myYouBotManipulator = Manipulator();
    myYouBotManipulator.moveArmJoints(input_angles);
    return 0;
}
