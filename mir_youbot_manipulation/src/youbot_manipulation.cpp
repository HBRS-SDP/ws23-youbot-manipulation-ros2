
/*
 * Copyright 2023 Bonn-Rhein-Sieg University
 *
 * ROS2 Jay Parikh, Kamendu Panchal, Chaitanya Gumudala, Mohsen Azizmalayeri.
 *
 */

#include "mir_youbot_manipulation/youbot_manipulation.hpp"

using namespace youbot;
using namespace manipulation_namespace;

Manipulator::Manipulator(const std::string &file_path):myArm("youbot-manipulator", file_path)
{
    EthercatMaster::getInstance("youbot-ethercat.cfg", file_path, true);
    myArm.doJointCommutation();
    myArm.calibrateManipulator();
    readYAML();
}

void Manipulator::readYAML() 
{
    auto ros2_path = ament_index_cpp::get_package_share_directory("mir_youbot_manipulation");
    string file_path = ros2_path + "/config/joint_limits.yaml";
    YAML::Node node = YAML::LoadFile(file_path);
    if (node["mir_youbot_manipulation"])
    {
        for (const auto &entry : node["mir_youbot_manipulation"])
        {
            if (entry["joint_limits"])
            {
                for (const auto &joint : entry["joint_limits"]["joints"])
                {
                    for (const auto &joint_entry : joint)
                    {
                        JointAngleSetpoint minimum_setpoint;
                        JointAngleSetpoint maximum_setpoint;
                        double minimum = joint_entry.second["min"].as<double>();
                        double maximum = joint_entry.second["max"].as<double>();
                        minimum_setpoint.angle = minimum * radian;
                        maximum_setpoint.angle = maximum * radian;
                        minimum_angles.push_back(minimum_setpoint);
                        maximum_angles.push_back(maximum_setpoint);
                    }
                }
            }

            if (entry["joint_compensations"])
            {
                for (const auto &joint : entry["joint_compensations"]["joints"])
                {
                    for (const auto &joint_entry : joint)
                    {
                        JointAngleSetpoint compensate_setpoint;
                        double compensate = joint_entry.second["compensate"].as<double>();
                        compensate_setpoint.angle = compensate * radian;
                        compensate_angles.push_back(compensate_setpoint);
                    }
                }
            }
        }
    }
}

void Manipulator::convertDegToRad(const std::vector<JointAngleSetpoint> &joint_angles_deg, std::vector<JointAngleSetpoint> &joint_angles_rad) 
{
    for (int i = 0; i < joint_angles_deg.size(); i++)
    {
        JointAngleSetpoint joint_angle_rad;
        joint_angle_rad.angle = (joint_angles_deg[i].angle.value() * M_PI / 180.0) * radian;
        joint_angles_rad.push_back(joint_angle_rad);
    }
}

bool Manipulator::validateInput(const std::vector<JointAngleSetpoint> &joint_angles_rad)
{
    if (joint_angles_rad.size() != minimum_angles.size() || joint_angles_rad.size() != maximum_angles.size())
    {
        std::cout << "Error: Input vector size does not match expected size" << false;
    }
    for (int i = 0; i < joint_angles_rad.size(); i++)
    {
        if (joint_angles_rad[i].angle.value() < minimum_angles[i].angle.value() || joint_angles_rad[i].angle.value() > maximum_angles[i].angle.value())
        {
            std::cout << "Joint angle " << i + 1 << " is out of range" << std::endl;
            return false;
        }
    }
    return true;
}

void Manipulator::convertJointAnglesToYoubotDriverConvention(const std::vector<JointAngleSetpoint> &joint_angles_rad, const std::vector<JointAngleSetpoint> &compensate_angles, std::vector<JointAngleSetpoint> &youbot_angles_set_point)
{
    for (int i = 0; i < joint_angles_rad.size(); i++)
    {
        JointAngleSetpoint youbot_driver_joint_angle;
        youbot_driver_joint_angle.angle = (joint_angles_rad[i].angle.value() + compensate_angles[i].angle.value()) * radian;
        youbot_angles_set_point.push_back(youbot_driver_joint_angle);
    }
}

vector<JointAngleSetpoint> Manipulator::convertDoubleToJointAngleSetpoint(const std::vector<double> &input_angle)
{
    vector<JointAngleSetpoint> youbot_angles_set_point;
    for (int i = 0; i < input_angle.size(); i++)
    {
        JointAngleSetpoint youbot_angle_set_point;
        youbot_angle_set_point.angle = input_angle[i] * radian;
        youbot_angles_set_point.push_back(youbot_angle_set_point);
    }
    return youbot_angles_set_point;
}

bool Manipulator::moveArmJoints(const std::vector<JointAngleSetpoint> &joint_angles_deg)
{
    vector<JointAngleSetpoint> joint_angles_rad;
    convertDegToRad(joint_angles_deg, joint_angles_rad);
    if (validateInput(joint_angles_rad))
    {
        vector<JointAngleSetpoint> youbot_angles_set_point;
        convertJointAnglesToYoubotDriverConvention(joint_angles_rad, compensate_angles, youbot_angles_set_point);
        for (int i = 0; i < youbot_angles_set_point.size(); i++)
        {
            std::cout << "Input joint " << i + 1 << " angle to the youbot : " << youbot_angles_set_point[i].angle.value() << std::endl;
        }
        // myArm.setJointData(youbot_angles_set_point);
        while (true)
        {
            // sleep(3);
            vector<JointSensedAngle> youbot_sensed_angles;
            // myArm.getJointData(youbot_sensed_angles);
            for (int i = 0; i < youbot_angles_set_point.size(); i++)
            {
                if (abs(youbot_sensed_angles[i].angle.value() - youbot_angles_set_point[i].angle.value()) <= 1e-4)
                {
                    return false;
                }
            }
            return true;
        }
    }
    else
    {
        return false;
    }
}

bool Manipulator::inverseKinematics(const KDL::Frame& target_pose, const KDL::Chain& chain, KDL::JntArray& joint_angles_return) 
{
    // Create FK and IK solvers
    double eps = 1e-6;
    int max_iter = 100;
    KDL::ChainFkSolverPos_recursive fk_solver(chain);
    KDL::ChainIkSolverVel_pinv ik_solver_vel(chain, eps, max_iter);
    KDL::ChainIkSolverPos_NR ik_solver(chain, fk_solver, ik_solver_vel, max_iter, eps);
    // Set up joint angles
    KDL::JntArray joint_angles(chain.getNrOfJoints());
    joint_angles.data.setZero();
    // Set up input and output
    KDL::JntArray joint_angles_out(chain.getNrOfJoints());
    // Perform inverse kinematics
    int ik_result = ik_solver.CartToJnt(joint_angles, target_pose, joint_angles_out);
    if (ik_result < 0) {
        std::cerr << "Inverse Kinematics failed. Result: " << ik_result << std::endl;
        return false;
    }
    // Print resulting joint angles
    std::cout << "Joint angles (rad): " << joint_angles_out.data.transpose() << std::endl;
    // Assign resulting joint angles to the output parameter
    joint_angles_return = joint_angles_out;
    return true;
}

bool Manipulator::forwardKinematics(const KDL::JntArray& joint_angles, const KDL::Chain& chain, KDL::Frame& target_pose) 
{
    // Check if the number of joint angles matches the chain's number of joints
    if (joint_angles.rows() != chain.getNrOfJoints()) {
        std::cerr << "Error: Number of joint angles does not match the chain's number of joints." << std::endl;
        return false;
    }
    // Create a forward kinematics solver
    KDL::ChainFkSolverPos_recursive fk_solver(chain);
    // Set up the input joint positions
    KDL::JntArray joint_positions(chain.getNrOfJoints());
    joint_positions.data = joint_angles.data;
    // Calculate the forward kinematics
    int fk_result = fk_solver.JntToCart(joint_positions, target_pose);
    if (fk_result < 0) {
        std::cerr << "Forward Kinematics failed. Result: " << fk_result << std::endl;
        return false;
    }
    // Print the resulting target pose
    std::cout << "Target Pose:\n" << target_pose << std::endl;
    return true;
}

//int main(int argc, char **argv)
//{
//     auto ethercat_path = ament_index_cpp::get_package_share_directory("youbot_driver");
//     string file_path = ethercat_path + "/config";
//     std::cout << file_path << std::endl;
//     vector<double> input_angles;
//     for (int i = 0; i < 5; i++)
//     {
//         double angle;
//         std::cout << "Enter joint " << i + 1 << " angle : ";
//         std::cin >> angle;
//         input_angles.push_back(angle);
//     }
//     Manipulator myYouBotManipulator = Manipulator(file_path);
//     vector<JointAngleSetpoint> joint_angles = myYouBotManipulator.convertDoubleToJointAngleSetpoint(input_angles);
//     myYouBotManipulator.moveArmJoints(joint_angles);
//     return 0;
//}