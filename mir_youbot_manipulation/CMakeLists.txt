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
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(youbot_driver)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp)
find_package(Boost REQUIRED COMPONENTS thread date_time filesystem system regex)


include_directories(
  include ${Boost_INCLUDE_DIR}
  ${YAML_CPP_VENDOR_INCLUDE_DIRS}
)

add_executable(youbot_manipulation
    src/main.cpp
    )
    
    ament_target_dependencies(youbot_manipulation 
    rclcpp
    std_msgs 
    geometry_msgs
    youbot_driver
    yaml_cpp_vendor
    ament_index_cpp
    )
    target_link_libraries(youbot_manipulation 
      ${Boost_LIBRARIES}
      ${youbot_driver_LIBRARIES}
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
