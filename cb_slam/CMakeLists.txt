cmake_minimum_required(VERSION 3.8)
project(cb_slam)

# find ament build system
find_package(ament_cmake REQUIRED)

# for getting package paths at runtime
find_package(slam_toolbox REQUIRED)
find_package(turtlebot3_gazebo REQUIRED)
find_package(turtlebot3_description REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rviz2 REQUIRED)
find_package(rclcpp REQUIRED) 

find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)



# install launch files
install(
  DIRECTORY 
  launch
  world
  map
  param
  DESTINATION share/${PROJECT_NAME}
)

add_executable(localization_score src/localization_score.cpp)

ament_target_dependencies(localization_score
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  ament_index_cpp
)

target_link_libraries(localization_score yaml-cpp)

install(TARGETS
  localization_score
  DESTINATION lib/${PROJECT_NAME}
)

# if you have RViz config files, e.g. rviz
# install(
#   DIRECTORY rviz
#   DESTINATION share/${PROJECT_NAME}
# )

ament_package()
