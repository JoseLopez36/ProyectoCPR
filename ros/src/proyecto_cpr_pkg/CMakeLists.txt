cmake_minimum_required(VERSION 3.0.2)
project(proyecto_cpr_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
  pcl_ros
  cv_bridge
  image_transport
  nav_msgs
  move_base
  tf2_ros
  geometry_msgs
)

add_message_files(
  FILES
  CmdActuadores.msg
)

add_service_files(
  FILES
  NavegarHaciaDestino.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/controlador_actuadores.py
  scripts/controlador_pure_pursuit.py
  scripts/slam_visual.py
  scripts/planificador_global.py
  scripts/planificador_local.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)