cmake_minimum_required(VERSION 2.8.3)
project(wam_srvs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation genmsg)

#######################################
## Declare ROS messages and services ##
#######################################

add_service_files(
  FILES
  JP_PID.srv
  JV_PID.srv
  TP_PID.srv
  CartPosMove.srv
  GravityComp.srv
  HapticSphere.srv
  Hold.srv
  HoldGains.srv
  Connect.srv
  JointMove.srv
  JointMoveBlock.srv
  OrtnMove.srv
  OrtnGains.srv
  OrtnSplitMove.srv
  PoseMove.srv
  CartVel.srv
  BHandFingerPos.srv
  BHandFingerVel.srv
  BHandGraspPos.srv
  BHandGraspVel.srv
  BHandSpreadPos.srv
  BHandSpreadVel.srv
  Teach.srv
  Play.srv
  BHandPinchPos.srv
  Link.srv
  ForceTorque.srv
  ForceTorqueTool.srv
  ForceTorqueBase.srv
  ForceTorqueToolTime.srv
  FollowPath.srv
  StaticForceEstimationwithG.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS message_runtime geometry_msgs
)
