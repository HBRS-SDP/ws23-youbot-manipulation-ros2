cmake_minimum_required(VERSION 3.5)
project(brics_actuator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(msg_files
	msg/CartesianPose.msg
	msg/CartesianTwist.msg
	msg/CartesianVector.msg
	msg/CartesianWrench.msg
	msg/JointAccelerations.msg
	msg/JointConstraint.msg
	msg/JointImpedances.msg
	msg/JointPositions.msg
	msg/JointTorques.msg
	msg/JointValue.msg
	msg/JointVelocities.msg
	msg/Poison.msg
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES geometry_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
