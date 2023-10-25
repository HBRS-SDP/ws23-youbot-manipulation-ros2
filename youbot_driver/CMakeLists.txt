cmake_minimum_required(VERSION 3.5)
project(youbot_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread date_time filesystem system regex)


list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

if(WIN32)
	add_definitions(-D_USE_MATH_DEFINES)    # get M_PI under Windows
	add_definitions(-DWIN32_LEAN_AND_MEAN)  # avoid inclusion of WinSock.h by windows.h, as we need WinSock2.h
	add_definitions(-DNOMINMAX)             # be able to use std::max, instead of windows defined functions
	add_definitions(/wd4800)                # disable warning: variable set to boolean value

	# need WinPcap library, using own cmake find file
	find_package(WinPcap REQUIRED)
endif()

if(WIN32)
	set(INCLUDE_DIRS
		include
		include/youbot_driver/soem/osal
		include/youbot_driver/soem/osal/win32
		include/youbot_driver/soem/oshw/win32
		${WINPCAP_INCLUDE_DIRS}
	)
else()
	set(INCLUDE_DIRS
		include
		include/youbot_driver/soem/osal
		include/youbot_driver/soem/oshw/linux
	)
endif()


## Note exported include dirs for install in cmake/config.cmake
include_directories(
	${INCLUDE_DIRS}
)


########### CppUnit and tests ###########
find_package(CppUnit)
if(CppUnit_FOUND)
	message(STATUS "SUCCESSFUL: CPP_UNIT found")
	add_subdirectory(src/testing)
else()
#	MESSAGE(STATUS "WARNING: CPP_UNIT not found")
endif()


########### Doxygen ###########
#SET(DOXYFILE_LATEX "NO")
#SET(DOXYFILE_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/doc)
#include(UseDoxygen)

########### Simple Open EtherCAT Master Library ###########
add_subdirectory(src/soem)

########### youBot display example application ###########
#add_subdirectory(src/display)

set(YOUBOT_DRIVER_SRC
  src/youbot/EthercatMaster.cpp
  src/youbot/EthercatMasterWithThread.cpp
  src/youbot/EthercatMasterWithoutThread.cpp
  src/generic/Logger.cpp
  src/generic/ConfigFile.cpp
  src/generic/PidController.cpp
  src/generic-joint/JointData.cpp
  src/generic-joint/JointTrajectory.cpp
  src/youbot/YouBotJoint.cpp
  src/youbot/JointLimitMonitor.cpp
  src/youbot/YouBotBase.cpp
  src/youbot/YouBotManipulator.cpp
  src/youbot/YouBotGripper.cpp
  src/youbot/YouBotGripperBar.cpp
  src/youbot/YouBotGripperParameter.cpp
  src/youbot/DataTrace.cpp
  src/youbot/GripperDataTrace.cpp
  src/youbot/YouBotJointParameter.cpp
  src/youbot/YouBotJointParameterReadOnly.cpp
  src/youbot/YouBotJointParameterPasswordProtected.cpp
  src/youbot/JointTrajectoryController.cpp
  src/base-kinematic/FourSwedishWheelOmniBaseKinematic.cpp
  src/base-kinematic/FourSwedishWheelOmniBaseKinematicConfiguration.cpp
)

add_library(YouBotDriver ${YOUBOT_DRIVER_SRC})
ament_target_dependencies(YouBotDriver
    rclcpp
    Boost
)
target_link_libraries(YouBotDriver
    soem
)
target_compile_definitions(YouBotDriver PRIVATE
    -DUSE_ROS_LOGGING
)


#install binary and lib
install(TARGETS YouBotDriver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

#copy header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

#copy config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)


#INSTALL(TARGETS YouBotDriver YouBotDriverShared RUNTIME DESTINATION bin LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)


#ADD_EXECUTABLE(YouBot_test
#  ${PROJECT_SOURCE_DIR}/Test_commutation.cpp
#)

#target_link_libraries(YouBot_test YouBotDriver)

ament_export_dependencies(
    rclcpp
    Boost
)
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(
  YouBotDriver
  soem
  rt
)

ament_package(
    CONFIG_EXTRAS cmake/config.cmake
)

