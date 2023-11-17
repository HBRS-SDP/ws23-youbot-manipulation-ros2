#include <iostream>
#include <vector>
#include <cmath>
#include "youbot_driver/youbot/EthercatMasterWithThread.hpp"
#include "youbot_driver/youbot/YouBotManipulator.hpp"
#include "youbot_driver/youbot/DataTrace.hpp"

const std::vector<double> minimum_angles = {-2.94961, -1.13446, -2.63545, -1.78896248, -2.879793};
const std::vector<double> maximum_angles = {2.94961, 1.5708, 2.54818, 1.78896248,  2.87979};
const std::vector<double> q_offsets = {
    (169.0 * M_PI) / 180.0,
    (65.0 * M_PI) / 180.0,
    (-151.0 * M_PI) / 180.0,
    (102.5 * M_PI) / 180.0,
    (65.0 * M_PI) / 180.0
};

using namespace youbot;
// add your path
std::string ethercat_config_path = "./config";

// Function to convert input angles from degress into radians
vector<double> convert_deg_to_rad(vector<double> deg_angles) {
    /*
    Converts degrees to radians.
    input: degrees in vector<double> as a list of angles
    output: radians in vector<double> stored in a list
    */
    vector<double> rad_angles;
    for (int i = 0; i < deg_angles.size(); i++) {
        rad_angles.push_back(deg_angles[i] * M_PI / 180.0);
    }
    return rad_angles;
}

//Function to check the input joint angles is within the range or not
bool validate_input(std::vector<double> joint_angles) {
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
// Function to convert double values to joint angle set points type
vector<JointAngleSetpoint> convert_double_to_joint_angle_setpoint(std::vector<double> joint_angles) {
    std::vector<JointAngleSetpoint> result;
    for (int i = 0; i < joint_angles.size(); i++) {
        double joint_angle = joint_angles[i];
        JointAngleSetpoint input_angle;
        input_angle.angle = joint_angle * radian;
        result.push_back(input_angle);
    }
    return result;
}

// Funtion to convert joint angles convention to youbot driver convention

vector<JointAngleSetpoint> convertJointAnglesToYouBotConvention() {

    int len = sizeof(inputAngles) / sizeof(inputAngles[0]);

    for(int i = 0; i < len; i++) {
        outputAngles[i] = inputAngles[i] - q_offsets[i];

        
    } 

    return *outputAngles;  
}

// Function to convert youbot driver convention to joint angles convention

double convertJointAnglesToJointConvention(double *inputAngles, double *outputAngles, double *q_offsets) {

    int len = sizeof(inputAngles) / sizeof(inputAngles[0]);
    for(int i = 0; i < len; i++) {
        outputAngles[i] = inputAngles[i] + q_offsets[i];
    }

    return *outputAngles;
}

// Funtion which takes input angles and move the robot to given configuration of joint angles



int main(int argc, char **argv) {
    
    // EthercatMaster::getInstance("youbot-ethercat.cfg", ethercat_config_path, true);
    // YouBotManipulator myArm("youbot-manipulator", ethercat_config_path);
    // myArm.doJointCommutation();=
    // myArm.calibrateManipulator();
    // double minimum_angles[] = {-2.94961, -1.13446, -2.63545, -1.78896248, -2.879793};
    // double maximum_angles[] = {2.94961, 1.5708, 2.54818, 1.78896248, 2.87979};
    
    // double q_offsets[5];
    // q_offsets[0] = (169.0 * M_PI) / 180.0;
    // q_offsets[1] = (65.0 * M_PI) / 180.0;
    // q_offsets[2] = (-151.0 * M_PI) / 180.0;
    // q_offsets[3] = (102.5 * M_PI) / 180.0;
    // q_offsets[4] = (65.0 * M_PI) / 180.0;
    

    // std::vector<JointAngleSetpoint> inputAngles;
    // std::vector<JointSensedAngle> sensedAngles;
    
    // // myArm.getJointData(sensedAngles);
    // // for (int i = 0; i < 5; i++) {
    // //     std::cout << sensedAngles[i].angle.value() - q_offsets[i] << std::endl;
    // // }
    


    // std::cout << "Enter joint angles(in radians)" << std::endl;

    // for (int i=0; i < 5; i++) {
    //     double angle;
    //     JointAngleSetpoint inputangle;
        
    //     std::cout << "Enter joint angle-" << i + 1 << " : ";
    //     std::cin >> angle;
    //     inputangle.angle = (angle + q_offsets[i]) * radian;

    //     if (isinRange(angle, minimum_angles[i], maximum_angles[i])) {
    //         inputAngles.push_back(inputangle);
    //     } else {
    //         std::cout << "Wrong limits" << std::endl;
    //         --i;
    //     }
    // }
    // std::cout << "Moving robot to the desired joint angles" << std::endl;
    // //myArm.setJointData(inputAngles);
    // sleep(5);
    // return 0;
}
