cmake_minimum_required(VERSION 3.0.2)
project(ai_module)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
   FILES
   SampleMsg.msg
#   Message2.msg
)

add_service_files(
   FILES
   ImageProc.srv
#   Service2.srv
)
## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES main_package_two
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



catkin_install_python(PROGRAMS
   scripts/start_image_processing_server.py
   scripts/sample_fun.py
   scripts/sample_fun_call.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



