#include <iostream>
#include <vector>
#include "youbot_driver/youbot/EthercatMasterWithThread.hpp"
#include "youbot_driver/youbot/YouBotManipulator.hpp"
#include "youbot_driver/youbot/DataTrace.hpp"

using namespace youbot;
// add your path
std::string ethercat_config_path = "/home/chaitanya/workspace_sdp/src/youbot_driver/config";

//Function to check the input joint angle is within the range or not
bool isinRange(double value, double minimum, double maximum){
    return (value >= minimum) && (value <= maximum);
}

int main(int argc, char **argv) {
    // EthercatMaster::getInstance("youbot-ethercat.cfg", ethercat_config_path, true);
    // YouBotManipulator myArm("youbot-manipulator", ethercat_config_path);
    // myArm.doJointCommutation();
    // myArm.calibrateManipulator();
    double minimum_angles[] = {-2.94961, -1.13446, -2.63545, -1.78896248, -2.879793};
    double maximum_angles[] = {2.94961, 1.5708, 2.54818, 1.78896248, 2.87979};

    std::vector<double> inputAngles;

    std::cout << "Enter joint angles(in radians)" << std::endl;

    for (int i; i < 5; i++) {
        double angle;
        std::cout << "Enter joint angle" << i + 1 << " : ";
        std::cin >> angle;

        if (isinRange(angle, minimum_angles[i], maximum_angles[i])) {
            inputAngles.push_back(angle);
        } else {
            std::cout << "Wrong limits" << std::endl;
            --i;
        }
    }
    
    std::cout << "Entered joint angles: ";
    for (const auto& angle : inputAngles) {
        std::cout << angle << " ";
    }
    std::cout << std::endl;
    return 0;
}
