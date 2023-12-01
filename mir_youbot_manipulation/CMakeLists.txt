cmake_minimum_required(VERSION 3.8)
project(mir_youbot_manipulation)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mir_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(youbot_driver)
find_package(yaml-cpp)
find_package(brics_actuator)
find_package(Boost REQUIRED COMPONENTS thread date_time filesystem system regex)


include_directories(
  include ${Boost_INCLUDE_DIR}

)

add_executable(youbot_manipulation
    src/youbot_manipulation.cpp
    src/youbot_manipulation_node.cpp
    )
    
    ament_target_dependencies(youbot_manipulation 
    rclcpp
    rclcpp_lifecycle
    std_msgs 
    geometry_msgs
    youbot_driver
    rclcpp_action
    ament_index_cpp
    brics_actuator
    mir_interfaces
    )
    target_link_libraries(youbot_manipulation 
      ${Boost_LIBRARIES}
      ${youbot_driver_LIBRARIES}
      ${brics_actuator_LIBRARIES}
      yaml-cpp
      
    )


install(
  DIRECTORY include/ 
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY config/ 
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  TARGETS youbot_manipulation
#   EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)



ament_package()
