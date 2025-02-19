cmake_minimum_required(VERSION 3.0.2)
project(bebop_control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
  tf
  custom_msgs
  gazebo_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs custom_msgs gazebo_msgs bebop_msgs nav_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

catkin_install_python(PROGRAMS
    src/bebop_open_loop_node.py
    src/bebop_closed_loop_node.py
    src/bebop_odometry_to_adjusted.py
    src/model_estimation.py
    src/model_comparison.py
    src/bebop_optitrack_to_adjusted.py
    src/security_node.py
    src/rviz_node.py
    src/bebop_PD_trajectory_cont.py
    src/bebop_open_loop_roll_pitch.py
    src/bebop_open_loop_pi_isa.py
    src/bebop_open_25.11.2024.py
    src/bebop_open_03.02.2025.py
    src/world_to_bebop.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )