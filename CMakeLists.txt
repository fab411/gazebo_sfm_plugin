cmake_minimum_required(VERSION 3.8)
project(gazebo_sfm_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(gazebo REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

include_directories(include)
include_directories(SYSTEM
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${GAZEBO_LIBRARY_DIRS}
)

add_library(PedestrianSFMPlugin SHARED src/PedestrianSFMPlugin.cpp)
target_include_directories(PedestrianSFMPlugin PUBLIC include)
ament_target_dependencies(PedestrianSFMPlugin 
  "rclcpp" 
  "gazebo" 
  "gazebo_ros"
  "geometry_msgs"
  "unique_identifier_msgs"
)
ament_export_libraries(PedestrianSFMPlugin)
target_link_libraries(PedestrianSFMPlugin ${GAZEBO_LIBRARIES}) #${Boost_LIBRARIES

add_library(PedestrianSFM2Plugin SHARED src/PedestrianSFM2Plugin.cpp)
target_include_directories(PedestrianSFM2Plugin PUBLIC include)
ament_target_dependencies(PedestrianSFM2Plugin 
  "rclcpp" 
  "gazebo" 
  "gazebo_ros"
  "geometry_msgs"
  "unique_identifier_msgs"
)
ament_export_libraries(PedestrianSFM2Plugin)
target_link_libraries(PedestrianSFM2Plugin ${GAZEBO_LIBRARIES}) #${Boost_LIBRARIES

add_library(PedestrianHSFMPlugin SHARED src/PedestrianHSFMPlugin.cpp)
target_include_directories(PedestrianHSFMPlugin PUBLIC include)
ament_target_dependencies(PedestrianHSFMPlugin 
  "rclcpp" 
  "gazebo" 
  "gazebo_ros"
  "geometry_msgs"
  "unique_identifier_msgs"
)
ament_export_libraries(PedestrianHSFMPlugin)
target_link_libraries(PedestrianHSFMPlugin ${GAZEBO_LIBRARIES}) #${Boost_LIBRARIES

add_library(PedestrianHSFM2Plugin SHARED src/PedestrianHSFM2Plugin.cpp)
target_include_directories(PedestrianHSFM2Plugin PUBLIC include)
ament_target_dependencies(PedestrianHSFM2Plugin 
  "rclcpp" 
  "gazebo" 
  "gazebo_ros"
  "geometry_msgs"
  "unique_identifier_msgs"
)
ament_export_libraries(PedestrianHSFM2Plugin)
target_link_libraries(PedestrianHSFM2Plugin ${GAZEBO_LIBRARIES}) #${Boost_LIBRARIES

add_library(VisualizeForcesPlugin SHARED src/VisualizeForcesPlugin.cpp)
target_include_directories(VisualizeForcesPlugin PUBLIC include)
ament_target_dependencies(VisualizeForcesPlugin 
  "rclcpp" 
  "gazebo" 
  "gazebo_ros"
  "geometry_msgs"
  "unique_identifier_msgs"
)
add_library(PedestrianMHSFMPlugin SHARED src/PedestrianMHSFMPlugin.cpp)
target_include_directories(PedestrianMHSFMPlugin PUBLIC include)
ament_target_dependencies(PedestrianMHSFMPlugin 
  "rclcpp" 
  "gazebo" 
  "gazebo_ros"
  "geometry_msgs"
  "unique_identifier_msgs"
)
ament_export_libraries(PedestrianMHSFMPlugin)
target_link_libraries(PedestrianMHSFMPlugin ${GAZEBO_LIBRARIES}) #${Boost_LIBRARIES
ament_export_libraries(VisualizeForcesPlugin)
target_link_libraries(VisualizeForcesPlugin ${GAZEBO_LIBRARIES}) #${Boost_LIBRARIES

set(msg_files
  "msg/UUID.msg"
  "msg/Point.msg"  
  "msg/Vector2.msg"
  "msg/Vector3.msg"
  "msg/Pose2.msg"
  "msg/Forces.msg"
  "msg/Obstacle.msg"
  "msg/ObstacleArray.msg"
  "msg/Person.msg"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs unique_identifier_msgs
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

ament_export_dependencies(rosidl_default_runtime)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

  target_link_libraries(PedestrianMHSFMPlugin "${cpp_typesupport_target}")
target_link_libraries(PedestrianHSFM2Plugin "${cpp_typesupport_target}")
target_link_libraries(PedestrianHSFMPlugin "${cpp_typesupport_target}")
target_link_libraries(PedestrianSFMPlugin "${cpp_typesupport_target}")
target_link_libraries(PedestrianSFM2Plugin "${cpp_typesupport_target}")
target_link_libraries(VisualizeForcesPlugin "${cpp_typesupport_target}")

install(TARGETS PedestrianSFMPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS PedestrianSFM2Plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS PedestrianHSFMPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS PedestrianHSFM2Plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS PedestrianMHSFMPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS VisualizeForcesPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
        DESTINATION include
)
install(DIRECTORY launch/
   DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY worlds/
   DESTINATION share/${PROJECT_NAME}/worlds
)
install(DIRECTORY models/
   DESTINATION share/${PROJECT_NAME}/models
)

# Create the libAttachModelPlugin.so library.
set(attach_model_plugin_name AttachModelPlugin)
add_library(${attach_model_plugin_name} SHARED
  src/AttachModelPlugin.cc
)
target_link_libraries(${attach_model_plugin_name}
  ${GAZEBO_LIBRARIES}
)

install(TARGETS ${attach_model_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install Python executables
install(PROGRAMS
  scripts/save_and_print_forces.py
  scripts/spawn_robot.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()


#/home/fab/ros2_humble/install/hunav_rviz2_panel/include/**
#/home/fab/ros2_humble/install/hunav_gazebo_wrapper/include/**
#/home/fab/ros2_humble/install/hunav_msgs/include/**
#/home/fab/ros2_humble/src/hunav_gazebo_wrapper/include/**
#/home/fab/ros2_humble/src/hunav_sim/hunav_agent_manager/include/**
#/home/fab/ros2_humble/src/hunav_sim/hunav_rviz2_panel/include/**