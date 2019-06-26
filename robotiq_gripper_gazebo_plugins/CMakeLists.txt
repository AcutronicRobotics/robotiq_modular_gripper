cmake_minimum_required(VERSION 3.5)
project(robotiq_gripper_gazebo_plugins)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX)
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -fpermissive -Wno-unused-parameter")
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -fpermissive -Wno-unused-parameter -Wno-header-guard -Wstatic-float-init")
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hrim_actuator_gripper_msgs REQUIRED)
find_package(hrim_actuator_gripper_srvs REQUIRED)

include_directories(include
  ${gazebo_ros_INCLUDE_DIRS}
  ${hrim_actuator_gripper_msgs_INCLUDE_DIRS}
  ${hrim_actuator_gripper_srvs_INCLUDE_DIRS}
)
link_directories(${gazebo_dev_LIBRARY_DIRS})

# robotiq finger gripper plugin
add_library(robotiq_gripper_gazebo_plugin SHARED
  src/RobotiqGripperPlugin.cpp
  src/spline.cpp
)

ament_target_dependencies(robotiq_gripper_gazebo_plugin
  "gazebo_ros"
  "rclcpp"
  "hrim_actuator_gripper_msgs"
  "hrim_actuator_gripper_srvs"
)

ament_export_libraries(robotiq_gripper_gazebo_plugin)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_dependencies(gazebo_ros)

ament_package()

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS
  robotiq_gripper_gazebo_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
