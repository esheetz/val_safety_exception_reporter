#---------------------------------------------------------------------------
#     Package Setup
#---------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.1)
project(val_safety_exception_reporter)

set(INSTALL_LIB_DIR ${CMAKE_INSTALL_PREFIX}/lib)

#------------------------------------------------------------------------
#     Build Setup
#------------------------------------------------------------------------
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dynamic_reconfigure
  geometry_msgs
  std_msgs
  message_generation
)

#------------------------------------------------------------------------
#     dynamic_reconfigure Param Config Setup
#------------------------------------------------------------------------
generate_dynamic_reconfigure_options(
  cfg/SafetyExceptionReporterParams.cfg
)
message("generated param config files!")

#------------------------------------------------------------------------
#     ROS Messages Setup
#------------------------------------------------------------------------
# generate messages
add_message_files(
  FILES
  AgentNotReady.msg
  CannotExecuteFootsteps.msg
  CannotExecuteMotion.msg
  CannotGetFootstepPlan.msg
  CannotGetMotionPlan.msg
  Collision.msg
  Collisions.msg
  GenericIssue.msg
  IgnoringCommands.msg
  InvalidCommand.msg
  InvalidFeedback.msg
  NotActionable.msg
  PauseCommand.msg
  SoftEStop.msg
  SoftEStopEndEffectorStreaming.msg
  SoftEStopJointState.msg
  SoftEStopJointStateDelta.msg
  StopCommand.msg
)

# dependencies for generated messages
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

message("generated messages!")

#------------------------------------------------------------------------
#     catkin Setup
#------------------------------------------------------------------------
catkin_package(
  CATKIN_DEPENDS message_runtime roscpp rospy geometry_msgs std_msgs
)
include_directories(${catkin_INCLUDE_DIRS})

#------------------------------------------------------------------------
#     Include Directories
#------------------------------------------------------------------------
include_directories ("${PROJECT_SOURCE_DIR}/..")
include_directories ("${PROJECT_SOURCE_DIR}")
include_directories ("${PROJECT_SOURCE_DIR}/nodes")
include_directories ("${PROJECT_SOURCE_DIR}/msg")

#---------------------------------------------------------------------------
#     Directory Selection
#---------------------------------------------------------------------------
add_subdirectory (nodes)
