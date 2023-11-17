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
const double PI = 3.141592;

// Dynamic config for ethercat_conf
string ethercat_config_path = "/home/chaitanya/workspace_sdp/src/youbot_driver/config";


vector<JointAngleSetpoint> convertDoubleToJointAngleSetpoint(vector<double> input_angles) {
    vector<JointAngleSetpoint> result;
    for (int i = 0; i < input_angles.size(); i++) {
        double joint_angle = input_angles[i];
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
    if (input_angles.size() != minimum_angles.size() || input_angles.size() != maximum_angles.size()) {
        std::cout << "Error: Input vector size does not match expected size" << std::endl;
        return false;    
    }
    for (int i = 0; i < input_angles.size(); i++) {
        if (input_angles[i] < minimum_angles[i] || input_angles[i] > maximum_angles[i]) {
            std::cout << "Joint angle " << i + 1<< " is out of range" << std::endl;
            return false;
        }
    }
    return true;
}

vector<double> convertJointAnglesToJointConvention(vector<double> input_angles, vector<double> compensate_angles) {
    vector<double> output_angles;
    output_angles.resize(input_angles.size());
    for(int i = 0; i < input_angles.size(); i++) {
        output_angles[i] = compensate_angles[i] + input_angles[i];
    }
    return output_angles;
}

// Funtion to convert joint angles convention to youbot driver convention
vector<double> convertJointAnglesToYouBotConvention(vector<double> input_angles, vector<double> compensate_angles) {
    vector<double> output_angles;
    output_angles.resize(input_angles.size());
    for(int i = 0; i < input_angles.size(); i++) {
        output_angles[i] = compensate_angles[i] - input_angles[i];
    }
    return output_angles;
}

void readYAML() {
    string file_path = "/home/chaitanya/workspace_sdp/src/mas_industrial_robotics/mir_manipulation/ws23-youbot-manipulation-ros2/mir_youbot_manipulation/config/joint_limits.yaml";
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

bool moveArmJoints(vector<double> input_angles){
    EthercatMaster::getInstance("youbot-ethercat.cfg", ethercat_config_path, true);
    YouBotManipulator myArm("youbot-manipulator", ethercat_config_path);
    myArm.doJointCommutation();
    myArm.calibrateManipulator();
	// Read YAML config file
	readYAML();
	
    vector<JointSensedAngle> youbot_current_angles;
    myArm.getJointData(youbot_current_angles);
    
	// Conversion of degree to radian
	vector<double> rad_angles;
	rad_angles = convertDegToRad(input_angles);
	// return rad_angles;
    
	// Validation
	if(validateInput(rad_angles)){
        vector<double> compensated_angles;
        compensated_angles = convertJointAnglesToJointConvention(rad_angles, compensate_angles);

        vector<JointAngleSetpoint> youbot_angle_set_point;
        youbot_angle_set_point = convertDoubleToJointAngleSetpoint(compensated_angles);
        for (int i = 0; i < youbot_angle_set_point.size(); i++) {
            cout << "Input joint " << i + 1 << " angle: " << youbot_angle_set_point[i].angle.value() << endl;
        }
        myArm.setJointData(youbot_angle_set_point);
        myArm.getJointData(youbot_current_angles);
        for (int i = 0; i < youbot_current_angles.size(); i++) {
            cout << "Current joint " << i  << " angle: " << youbot_current_angles[i].angle.value() - compensate_angles[i] << endl;
        }
        sleep(5);
        return true;
	} else {
		return false;
	}
}

int main(int argc, char **argv) {
    
   	   
	vector<double> input_angles;
    for (int i = 0; i < 5; i++) {
        double angle;
        cout << "Enter joint " << i + 1 << " angle : ";
        cin >> angle;
        input_angles.push_back(angle);
    }
    // vector<double> output_angles;
	// output_angles = moveArmJoints(input_angles);
    // for (int i = 0; i < output_angles.size(); i++) {
    //     cout << output_angles[i] << endl;
    // }
    moveArmJoints(input_angles);
    return 0;
}