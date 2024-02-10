/*
 * Copyright 2023 Bonn-Rhein-Sieg University
 *
 * ROS2 Jay Parikh, Kamendu Panchal, Chaitanya Gumudala, Mohsen Azizmalayeri.
 *
 */

#include "mir_youbot_manipulation/youbot_manipulation.hpp"

using namespace youbot;
using namespace manipulation_namespace;

// Manipulator::Manipulator()
// {
//   readYAML();
// }

Manipulator::Manipulator(const std::string &file_path)
    : myArm("youbot-manipulator", file_path)
{
  EthercatMaster::getInstance("youbot-ethercat.cfg", file_path, true);
  myArm.doJointCommutation();
  myArm.calibrateManipulator();
  readYAML();
}

void Manipulator::readYAML()
{
  auto ros2_path =
      ament_index_cpp::get_package_share_directory("mir_youbot_manipulation");
  string file_path = ros2_path + "/config/joint_limits.yaml";
  YAML::Node node = YAML::LoadFile(file_path);
  if (node["mir_youbot_manipulation"])
  {
    for (const auto &entry : node["mir_youbot_manipulation"])
    {
      if (entry["arm_configuration"])
      {
        number_of_joints = entry["arm_configuration"]["number_of_joints"].as<int>();
      }

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

bool Manipulator::validateInput(const std::vector<JointAngleSetpoint> &joint_angles_rad)
{
  if (joint_angles_rad.size() != number_of_joints ||
      minimum_angles.size() != number_of_joints ||
      maximum_angles.size() != number_of_joints)
  {
    std::cout << "Error: Input vector size does not match expected size" << false;
  }

  for (int i = 0; i < joint_angles_rad.size(); i++)
  {
    if (joint_angles_rad[i].angle.value() < minimum_angles[i].angle.value() ||
        joint_angles_rad[i].angle.value() > maximum_angles[i].angle.value())
    {
      std::cout << "Joint angle " << i + 1 << " is out of range" << std::endl;
      return false;
    }
  }
  return true;
}

void Manipulator::convertJointAnglesToYoubotDriverConvention(
    const std::vector<JointAngleSetpoint> &joint_angles_rad,
    const std::vector<JointAngleSetpoint> &compensate_angles,
    std::vector<JointAngleSetpoint> &youbot_angles_set_point)
{
  for (int i = 0; i < joint_angles_rad.size(); i++)
  {
    JointAngleSetpoint youbot_driver_joint_angle;
    youbot_driver_joint_angle.angle =
        (joint_angles_rad[i].angle.value() + compensate_angles[i].angle.value()) * radian;
    youbot_angles_set_point.push_back(youbot_driver_joint_angle);
  }
}

void Manipulator::convertJointAnglesToYoubotStoreConvention(
    const std::vector<JointSensedAngle> &joint_angles_rad,
    const std::vector<JointAngleSetpoint> &compensate_angles,
    std::vector<JointSensedAngle> &youbot_angles_set_point)
{
  for (int i = 0; i < joint_angles_rad.size(); i++)
  {
    JointSensedAngle youbot_store_joint_angle;
    youbot_store_joint_angle.angle =
        (joint_angles_rad[i].angle.value() - compensate_angles[i].angle.value()) * radian;
    youbot_angles_set_point.push_back(youbot_store_joint_angle);
  }
}

vector<JointAngleSetpoint> Manipulator::convertDoubleToJointAngleSetpoint(
    const std::vector<double> &input_angle)
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

bool Manipulator::moveArmJoints(const std::vector<JointAngleSetpoint> &joint_angles_rad)
{
  for (int i = 0; i < joint_angles_rad.size(); i++)
  {
    std::cout << "Input " << i + 1
              << " angle to the youbot : " << joint_angles_rad[i].angle.value()
              << std::endl;
  }
  if (validateInput(joint_angles_rad))
  {
    vector<JointAngleSetpoint> youbot_angles_set_point;
    convertJointAnglesToYoubotDriverConvention(joint_angles_rad, compensate_angles,
                                               youbot_angles_set_point);
    myArm.setJointData(youbot_angles_set_point);
    while (true)
    {
      sleep(3);
      vector<JointSensedAngle> youbot_sensed_angles;
      myArm.getJointData(youbot_sensed_angles);
      vector<JointSensedAngle> youbot_sensed_angles_set_point;
      convertJointAnglesToYoubotStoreConvention(youbot_sensed_angles, compensate_angles,
                                                youbot_sensed_angles_set_point);

      for (int i = 0; i < youbot_sensed_angles_set_point.size(); i++)
      {
        std::cout << "Sensed joint " << i + 1 << " angle of the youbot : "
                  << youbot_sensed_angles_set_point[i].angle.value() << std::endl;
      }
      for (int i = 0; i < youbot_angles_set_point.size(); i++)
      {
        if (abs(youbot_sensed_angles_set_point[i].angle.value() -
                youbot_angles_set_point[i].angle.value()) <= 1e-4)
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
  return true;
}

bool Manipulator::inverseKinematics(const KDL::Frame &target_pose,
                                    const KDL::Chain &chain,
                                    KDL::JntArray &joint_angles_return)
{
  KDL::JntArray joint_angles(chain.getNrOfJoints());
  KDL::JntArray joint_angles_out(chain.getNrOfJoints());
  KDL::ChainFkSolverPos_recursive fk_solver(chain);
  KDL::ChainIkSolverPos_LMA ik_solver(chain);

  int ik_result = ik_solver.CartToJnt(joint_angles, target_pose, joint_angles_out);
  if (ik_result < 0)
  {
    std::cerr << "Inverse Kinematics failed. Result: " << ik_result << std::endl;
    return false;
  }
  std::cout << "Joint angles (rad): " << joint_angles_out.data.transpose() << std::endl;
  joint_angles_return = joint_angles_out;
  return true;
}

bool Manipulator::forwardKinematics(const KDL::JntArray &joint_angles,
                                    const KDL::Chain &chain, KDL::Frame &target_pose)
{
  if (joint_angles.rows() != chain.getNrOfJoints())
  {
    std::cerr
        << "Error: Number of joint angles does not match the chain's number of joints."
        << std::endl;
    return false;
  }
  KDL::ChainFkSolverPos_recursive fk_solver(chain);
  KDL::JntArray joint_positions(chain.getNrOfJoints());
  joint_positions.data = joint_angles.data;
  int fk_result = fk_solver.JntToCart(joint_positions, target_pose);
  if (fk_result < 0)
  {
    std::cerr << "Forward Kinematics failed. Result: " << fk_result << std::endl;
    return false;
  }
  std::cout << "Position: " << target_pose.p.x() << " " << target_pose.p.y() << " "
            << target_pose.p.z() << std::endl;
  double yaw, pitch, roll;
  target_pose.M.GetRPY(roll, pitch, yaw);
  // std::cout << "Orientation: " << roll << " " << pitch << " " << yaw << std::endl;
  double qx, qy, qz, qw;
  target_pose.M.GetQuaternion(qx, qy, qz, qw);
  std::cout << "Quaternion: " << qx << " " << qy << " " << qz << " " << qw << std::endl;
  return true;
}

double Manipulator::calculateVelocityProfile(const double &amplitude,
                                             const double &start_pose,
                                             const double &target_pose,
                                             const double &current_pose)
{
  return amplitude * (target_pose - current_pose);  
  // if(target_pose/1.1 >= current_pose) // working good!!
  // {
  //   return amplitude * std::sin((M_PI * current_pose)/ (target_pose - start_pose));
  // }
  // else
  // {
  //   return 0.0;
  // }
  
}




bool Manipulator::moveArmJointsVelocity(const KDL::Chain &chain,
                                        const KDL::Frame &target_pose,
                                        const double amplitude)
{
  // Initialize variables
  std::vector<JointSensedAngle> youbot_driver_start_angles;
  std::vector<JointSensedAngle> youbot_driver_current_angles;
  std::vector<JointAngleSetpoint> youbot_driver_target_angles;
  std::vector<JointSensedAngle> youbot_store_start_angles;
  std::vector<JointSensedAngle> youbot_store_current_angles;
  std::vector<JointAngleSetpoint> youbot_store_target_angles;
  std::vector<JointVelocitySetpoint> joint_velocities_setpoint;

  KDL::Frame start_pose;
  KDL::Frame current_pose;
  KDL::ChainIkSolverVel_pinv solver(chain);
  KDL::Twist desired_twist;
  double velocity_x, velocity_y, velocity_z;
  double tolerance = 0.01;
  KDL::JntArray start_angles(chain.getNrOfJoints());
  KDL::JntArray current_angles(chain.getNrOfJoints());
  KDL::JntArray target_angles(chain.getNrOfJoints());
  KDL::JntArray joint_velocities(chain.getNrOfJoints());

  myArm.getJointData(youbot_driver_start_angles);
  inverseKinematics(target_pose, chain, target_angles);

  convertJointAnglesToYoubotStoreConvention(youbot_driver_start_angles, compensate_angles,
                                            youbot_store_start_angles);
  for (int i = 0; i < youbot_store_start_angles.size(); i++)
  {
    start_angles(i) = youbot_store_start_angles[i].angle.value();
  }
  forwardKinematics(start_angles, chain, start_pose);
  current_pose = start_pose;

  while ((target_pose.p - current_pose.p).Norm() > 1e-02)
  {
    youbot_store_current_angles.clear();
    joint_velocities_setpoint.clear();
    
    velocity_x = calculateVelocityProfile(amplitude, start_pose.p.x(), target_pose.p.x(), current_pose.p.x());
    velocity_y = calculateVelocityProfile(amplitude, start_pose.p.y(), target_pose.p.y(), current_pose.p.y());
    velocity_z = calculateVelocityProfile(amplitude, start_pose.p.z(), target_pose.p.z(), current_pose.p.z());

    std::cout << "Velocity x : " << velocity_x << " Velocity y : " << velocity_y
              << " Velocity z : " << velocity_z << std::endl;
    if ((target_pose.p - current_pose.p).Norm() <= 1e-02)
    {
        desired_twist.vel.x(0.0);
        desired_twist.vel.y(0.0);
        desired_twist.vel.z(0.0);
    }
    desired_twist.vel.x(velocity_x);
    desired_twist.vel.y(velocity_y);
    desired_twist.vel.z(velocity_z);

    solver.CartToJnt(current_angles, desired_twist, joint_velocities);

    for (int i = 0; i < joint_velocities.rows(); i++)
    {
      JointVelocitySetpoint joint_velocity_setpoint;
      joint_velocity_setpoint.angularVelocity = joint_velocities(i) * radian_per_second;
      joint_velocities_setpoint.push_back(joint_velocity_setpoint);
      double velocity_rad_per_s = joint_velocities(i);
      std::cout << "Joint " << i + 1 << " velocity : " << velocity_rad_per_s << std::endl;
    }

    myArm.setJointData(joint_velocities_setpoint);

    myArm.getJointData(youbot_driver_current_angles);
    convertJointAnglesToYoubotStoreConvention(
        youbot_driver_current_angles, compensate_angles, youbot_store_current_angles);

    for (int i = 0; i < youbot_store_current_angles.size(); i++)
    {
      current_angles(i) = youbot_store_current_angles[i].angle.value();
    }
    forwardKinematics(current_angles, chain, current_pose);
    usleep(10000);
  }
  return true;
}