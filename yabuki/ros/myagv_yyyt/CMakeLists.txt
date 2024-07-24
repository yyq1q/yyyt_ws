cmake_minimum_required(VERSION 3.0.2)
project(myagv_yyyt)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf2
  tf2_ros
  tf2_sensor_msgs
  turtlesim
  message_generation
  std_msgs
  sensor_msgs
  nav_msgs
  tf2_geometry_msgs
  pcl_ros
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(set_goal src/set_goal.cpp)
add_dependencies(set_goal
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(set_goal
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  Eigen3::Eigen 
)

add_executable(navigation src/navigation.cpp)
add_dependencies(navigation
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(navigation
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  Eigen3::Eigen 
)