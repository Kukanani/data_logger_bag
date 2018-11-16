cmake_minimum_required(VERSION 2.8.3)
project(data_logger_bag)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roslaunch
  rospy
  std_msgs
)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  LoggerSettings.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetSettings.srv
)

generate_messages(
  DEPENDENCIES std_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test
  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)