#include <iostream>
#include "youbot_driver/youbot/EthercatMasterWithThread.hpp"
#include "youbot_driver/youbot/YouBotManipulator.hpp"
#include "youbot_driver/youbot/DataTrace.hpp"

using namespace youbot;
// add your path
std::string ethercat_config_path = "/home/chaitanya/workspace_sdp/src/youbot_driver/config";
int main(int argc, char **argv) {
    std::cout << "Hello World"<< std::endl;
    EthercatMaster::getInstance("youbot-ethercat.cfg", ethercat_config_path, true);

    YouBotManipulator myArm("youbot-manipulator", ethercat_config_path);
    myArm.doJointCommutation();
    myArm.calibrateManipulator();
    return 0;
}
