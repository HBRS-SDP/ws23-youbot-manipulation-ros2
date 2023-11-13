#include <iostream>
#include <vector>
#include <cmath>
#include "youbot_driver/youbot/EthercatMasterWithThread.hpp"
#include "youbot_driver/youbot/YouBotManipulator.hpp"
#include "youbot_driver/youbot/DataTrace.hpp"

using namespace youbot;
// add your path
std::string ethercat_config_path = "./config";

// Function to convert input angles from degress into radians

//Function to check the input joint angles is within the range or not

// Funtion to convert joint angles convention to youbot driver convention

vector<double> convertJointAnglesToYouBotConvention(vector<double> inputAngles) {

    

    int len = inputAngles.size();

    vector<double> outputAngles[len];

    std::cout<<"Length of input angles: "<<len<<std::endl;


    for(int i = 0; i < len; i++) {
        outputAngles[i] = inputAngles[i] - q_offsets[i];
        
    } 

    return outputAngles;  
    }

// Function to convert youbot driver convention to joint angles convention

vector<double> convertJointAnglesToJointConvention(double *inputAngles, double *outputAngles, double *q_offsets) {


    // for(int i = 0; i <  inputAngles.size(); i++) {
    //     outputAngles[i] = inputAngles[i] + q_offsets[i];
    // }

    // return *outputAngles;
}

// Funtion which takes input angles and move the robot to given configuration of joint angles

bool isinRange(double value, double minimum, double maximum){
    return (value >= minimum) && (value <= maximum);
}

int main(int argc, char **argv) {
    // EthercatMaster::getInstance("youbot-ethercat.cfg", ethercat_config_path, true);
    // YouBotManipulator myArm("youbot-manipulator", ethercat_config_path);
    // myArm.doJointCommutation();=
    // myArm.calibrateManipulator();
    double minimum_angles[] = {-2.94961, -1.13446, -2.63545, -1.78896248, -2.879793};
    double maximum_angles[] = {2.94961, 1.5708, 2.54818, 1.78896248, 2.87979};
    // double q_offsets[] = {2.94961, 1.5708, 2.54818, 1.78896248, 2.87979};
    
    double q_offsets[5];
    q_offsets[0] = (169.0 * M_PI) / 180.0;
    q_offsets[1] = (65.0 * M_PI) / 180.0;
    q_offsets[2] = (-151.0 * M_PI) / 180.0;
    q_offsets[3] = (102.5 * M_PI) / 180.0;
    q_offsets[4] = (65.0 * M_PI) / 180.0;
    

    std::vector<JointAngleSetpoint> inputAngles;
    std::vector<JointSensedAngle> sensedAngles;
    
    // myArm.getJointData(sensedAngles);
    // for (int i = 0; i < 5; i++) {
    //     std::cout << sensedAngles[i].angle.value() - q_offsets[i] << std::endl;
    // }
    


    std::cout << "Enter joint angles(in radians)" << std::endl;

    for (int i=0; i < 5; i++) {
        double angle;
        JointAngleSetpoint inputangle;
        
        std::cout << "Enter joint angle-" << i + 1 << " : ";
        std::cin >> angle;
        inputangle.angle = (angle + q_offsets[i]) * radian;

        if (isinRange(angle, minimum_angles[i], maximum_angles[i])) {
            inputAngles.push_back(inputangle);
        } else {
            std::cout << "Wrong limits" << std::endl;
            --i;
        }
    }
    std::cout << "Moving robot to the desired joint angles" << std::endl;
    //myArm.setJointData(inputAngles);
    sleep(5);
    return 0;
}
