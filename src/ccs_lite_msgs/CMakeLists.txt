cmake_minimum_required(VERSION 3.0.2)
project(ccs_lite_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
   DIRECTORY msg
   FILES
   	Joystick.msg
   	Axis.msg
   	EnclosureStatus.msg
   	Door.msg
   	LidarEnclosureCmd.msg
   	MirEnclosureCmd.msg
   	GpsData.msg
   	UltrasonicData.msg
   	TemperatureHumidityData.msg
   	ProximityData.msg
   	PmsData.msg
   	CcsLiteData.msg
   	SprayGunCmd.msg
   	CcsLiteCmd.msg
   	SystemStatusData.msg
        EmergencyData.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ccs_lite_msgs
   CATKIN_DEPENDS message_runtime std_msgs
#  DEPENDS system_lib
)

#include_directories(
# include
#  ${catkin_INCLUDE_DIRS}
#)





