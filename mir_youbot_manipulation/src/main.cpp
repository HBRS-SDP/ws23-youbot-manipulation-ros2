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

//#include "mir_youbot_manipulation/youbot_manipulation.hpp"

using namespace std;
using namespace youbot;

vector<double> minimum_angles;
vector<double> maximum_angles;
vector<double> compensate_angles;
const double PI = 3.141592

// Dynamic config for ethercat_conf
std::string ethercat_config_path = "./config";

vector<JointAngleSetpoint> convertDoubleToJointAngleSetpoint(vector<double> input_angles) {
    vector<JointAngleSetpoint> result;
    for (int i = 0; i < joint_angles.size(); i++) {
        double joint_angle = joint_angles[i];
        JointAngleSetpoint input_angle;
        input_angle.angle = joint_angle * radian;
        result.push_back(input_angle);
    }
    return result;
}

vector<double> convertDegToRad(vector<double> deg_angles) {
    vector<double> rad_angles;
    for (int i = 0; i < deg_angles.size(); i++) {
        rad_angles.push_back(deg_angles[i] * PI / 180.0);
    }
    return rad_angles;
}

bool validateInput(vector<double> input_angles) {
    if (joint_angles.size() != minimum_angles.size() || joint_angles.size() != maximum_angles.size()) {
        std::cout << "Error: Input vector size does not match expected size" << std::endl;
        return false;    
    }
    for (int i = 0; i < joint_angles.size(); i++) {
        if (joint_angles[i] < minimum_angles[i] || joint_angles[i] > maximum_angles[i]) {
            std::cout << "Joint angle " << i + 1<< " is out of range" << std::endl;
            return false;
        }
    }
    return true;
}

vector<JointAngleSetpoint> convertJointAnglesToJointConvention(vector<JointAngleSetpoint> input_angles, vector<JointAngleSetpoint> vector<double> compensate_angles) {
    vector<JointAngleSetpoint> output_angles;
	int len = sizeof(input_angles) / sizeof(input_angles);
    for(int i = 0; i < len; i++) {
        output_angles[i] = input_angles[i] + compensate_angles[i];
    }
    return output_angles;
}

vector<JointAngleSetpoint> convertJointAnglesToYouBotConvention(vector<double> input_angles, vector<double> compensate_angles) {
    vector<JointAngleSetpoint> output_angles;
	int len = sizeof(input_angles) / sizeof(input_angles[0]);
    for(int i = 0; i < len; i++) {
        output_angles[i] = input_angles[i] - compensate_angles[i];
    } 
    return output_angles;  
}

void readYAML() {
    string file_path = "../config/joint_limits.yaml";
    YAML::Node node = YAML::LoadFile(file_path);

    if (node["mir_youbot_manipulation"]) {
        YAML::Node youbot_manipulation = node["mir_youbot_manipulation"];
        YAML::Node joint_angles = youbot_manipulation["joint_angles"];

        for (const auto& joint : joint_angles) {
            double minimum = joint["min"].as<double>();
            double maximum = joint["max"].as<double>();
            double compensate = joint["compensate"].as<double>();
            
            // Push the values into the global vectors
            minimum_angles.push_back(minimum);
            maximum_angles.push_back(maximum);
            compensate_angles.push_back(compensate);
        }
    }
}

bool moveJointArms(vector<double> input_angles){
	// Read YAML config file
	readYAML();
	
	// Conversion of degree to radian
	vector<double> rad_angles;
	rad_angles = convertDegToRad(input_angles);
	
	// Validation
	if(validateInput(rad_angles)){
		// Conversion to JointAngleSetpoint
		vector<JointAngleSetpoint> joint_angle_set_point;
		joint_angle_set_point = convertDoubleToJointAngleSetpoint(rad_angles)
		
		vector<JointAngleSetpoint> compensate_angle_set_point;
		compensate_angle_set_point = convertDoubleToJointAngleSetpoint(compensate_angles)
		
		// Add offset
		vector<JointAngleSetpoint> youbot_angle_set_point;
		youbot_angle_set_point = convertJointAnglesToJointConvention(joint_angle_set_point, compensate_angles_set_point)
		
		// Control loop
		vector<JointSensedAngle> sensed_youbot_angles;
		vector<JointSensedAngle> compensate_sensed_youbot_angles;
		myArm.getJointData(sensed_youbot_angles);
		compensate_sensed_youbot_angles = convertJointAnglesToYouBotConvention(sensed_youbot_angles)
		// Compare compensated sensed_angle with joint_angle_set_point
		while(true){ 
			myArm.setJointData(youbot_angle_set_point);
			sleep(5);
		}
		return true;
	} else {
		return false;
	}
}

int main(int argc, char **argv) {
    
	/*
    EthercatMaster::getInstance("youbot-ethercat.cfg", ethercat_config_path, true);
    YouBotManipulator myArm("youbot-manipulator", ethercat_config_path);
    myArm.doJointCommutation();=
    myArm.calibrateManipulator();
    */
	vector<double> input_angles = {0.0, 0.0, 0.0, 0.0, 0.0};
	result = moveJointArms(input_angles);
    return result;
}
