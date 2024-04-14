cmake_minimum_required(VERSION 3.0.2)
project(user_control)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  xacro
  gazebo_msgs
  tf
  tf2_ros
  sensor_msgs
  message_filters

  moveit_core
  moveit_msgs
  moveit_visual_tools
  moveit_ros_move_group
  moveit_ros_planning_interface
  moveit_ros_perception

  cv_bridge  
)
find_package(OpenCV REQUIRED)


# Eigen库
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES user_control
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib

)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(rcm_keyboard src/rcm_keyboard.cpp)
target_link_libraries(rcm_keyboard ${catkin_LIBRARIES})

add_executable(rcm_joystick src/rcm_joystick.cpp)
target_link_libraries(rcm_joystick ${catkin_LIBRARIES})

add_executable(rcm_hand src/rcm_hand.cpp)
target_link_libraries(rcm_hand ${catkin_LIBRARIES})

