#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "status_check/Status_data.h"
#include "status_check/stats.h"
#include <sstream>
#include <bits/stdc++.h>
#include <string>  
#include <iostream>
using namespace std;

status_check::stats mir;
status_check::stats arm; 
status_check::stats ai;
status_check::stats lidar;
std::string mir_battery = "5.0";
std::string gps = "";
float main_battery = 5.0; 
std::string temp = "0.0"; 
std::string hum = "0.0 %"; 
std::string mir_state = "";
status_check::stats sqe_sensors;
//sqe_sensors.ready = true;
//sqe_sensors.running = true;
//sqe_sensors.fatalerr = false;
//sqe_sensors.remarks = "Temperature, Humidity"; 
bool sqe;
void restartStatusCheckNode(ros::NodeHandle &n);
template<class T>
std::string toString(const T &value){
    std::ostringstream os;
    os << value;
    return os.str();
}

void mirCallback(const std_msgs::String::ConstPtr& msg)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  mir_state = msg->data;
  if(mir_state == "Ready"){
      mir.ready = true;
      mir.running = false;
      mir.fatalerr = false;
      mir.remarks = "ready to run" ;  
  }
  else if(mir_state == "Executing"){
      mir.ready = true;
      mir.running = true;
      mir.fatalerr = false;
      mir.remarks = "MIR executing a autonomously planned path" ;    
  
  }
  else if(mir_state == "Error"){
      mir.ready = false;
      mir.running = false;
      mir.fatalerr = true;
      mir.remarks = "MIR is in error state - obstacles" ;     
  }
  else if(mir_state == "Pause"){
      mir.ready = true;
      mir.running = false;
      mir.fatalerr = false;
      mir.remarks = "MIR is paused" ;     
  }
  else if(mir_state == "EmergencyStop"){
      mir.ready = false;
      mir.running = false;
      mir.fatalerr = false;
      mir.remarks = "MIR is in emergency state" ;     
  }
  else{
      mir.ready = false;
      mir.running = true;
      mir.fatalerr = false;
      mir.remarks = "Unknown state" ;     
  }
}

/*void armCallback(const status_check::stats::ConstPtr& msg)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  arm = *msg;
}*/

//void aiserviceCallback(const status_check::stats::ConstPtr& msg)
//{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
//  ai = *msg; 
//}

void lidarRCallback(const status_check::stats::ConstPtr& msg)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  lidar= *msg;
}

void mirbatteryCallback(const std_msgs::String::ConstPtr& msg){
  cout<<"IM calling mir battery"<<endl;
  mir_battery = msg->data;
}

void gpsCallback(const std_msgs::String::ConstPtr& msg){
  gps = msg->data;
}

void mainbatteryCallback(const std_msgs::Float32::ConstPtr& msg){
  main_battery = msg->data;
}

void tempCallback(const std_msgs::Float32::ConstPtr& msg){
  temp = "";
  int val = (int)((msg->data)*100)/100;
  int dec = (int)(msg->data)%10;
  temp = std::to_string(val).append(".");
  temp = temp.append(std::to_string(dec));
  temp = temp.append(" C");
}

void humCallback(const std_msgs::Float32::ConstPtr& msg){
  hum = "";
  int val = (int)((msg->data)*100)/100;
  int dec = (int)(msg->data)%10;
  hum = std::to_string(val).append(".");
  hum = hum.append(std::to_string(dec));
  hum = hum.append(" %");
}

void sqeCallback(const std_msgs::Bool::ConstPtr& msg)
{
 // ROS_INFO("I heard: [%s]", msg->data.c_str());
 sqe = (*msg).data;
 if (sqe){
   sqe_sensors.ready = true;
   sqe_sensors.running = true;
   sqe_sensors.fatalerr = false;
   sqe_sensors.remarks = "Temperature, Humidity, OFS, Digital microscope, IR and Ultrasonic working status " ;
 } 
 else{
   sqe_sensors.ready = true;
   sqe_sensors.running = true;
   sqe_sensors.fatalerr = false;
   sqe_sensors.remarks = "check SQE connection" ;
 }
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "status_check_node");
  cout<<"im here"<<endl;
  ros::NodeHandle n;

  ros::Publisher status_pub = n.advertise<status_check::Status_data>("status_check_data", 10);
  cout<<"im here_2"<<endl;
  ros::Subscriber mir_sub = n.subscribe("/mir_state", 10, mirCallback);

  //ros::Subscriber lidar_sub = n.subscribe("lidar_status", 10, lidarRCallback);#not going to use it
  cout<<"im here_3"<<endl;
  ros::Subscriber mir_batt_sub = n.subscribe("/mir_battery_percentage",10,mirbatteryCallback);
  cout<<"im here_4"<<endl;
  ros::Subscriber main_batt_sub = n.subscribe("/pms_battery",10,mainbatteryCallback);

  ros::Subscriber temp_sub = n.subscribe("/coretemp",10,tempCallback);

  ros::Subscriber hum_sub = n.subscribe("/corehum",10,humCallback);
 
  ros::Subscriber gps_sub = n.subscribe("coregps",10,gpsCallback);

  //ros::Subscriber arm_sub = n.subscribe("arm_status", 10, armCallback);
  
  //ros::Subscriber ai_sub = n.subscribe("ai_service_status", 10, aiserviceCallback);
  
 // ros::Subscriber sqe_sub = n.subscribe("sqe_sensor_status", 10, sqeCallback);#not going to use it
 

  ros::Rate loop_rate(10);

  
  while (ros::ok())
  {
    bool m = false; 
    if(n.hasParam("axalta/ccscore/dashboard/RESTART_CCSCORE_TRIGGER")){
      n.getParam("axalta/ccscore/dashboard/RESTART_CCSCORE_TRIGGER",m);
    }
    if(m){
      restartStatusCheckNode(n);
    }
    
    status_check::Status_data msg;
    bool b = false;
    msg.mir = mir;
    //msg.arm = arm; //newly added
    if(n.hasParam("axalta/ccscore/arm_service/ARM_STARTUP")){
      n.getParam("axalta/ccscore/arm_service/ARM_STARTUP",b);
    }
    if(b){
        arm.ready = true;
        arm.running = true;
        arm.fatalerr = false;
        arm.remarks = "arm started";
    }
    else{
        arm.ready = false;
        arm.running = false;
        arm.fatalerr = false;
        arm.remarks = "arm not started";
    }
    ai.ready = true;
    ai.running = true;
    ai.fatalerr = false;
    ai.remarks = "ccs core started";
    msg.arm = arm;
    msg.ainode = ai;
    //std::string s = "Process started";
    std::string s;
    if(n.hasParam("axalta/ccscore/dashboard/CURRENT_PROCESS")){
      n.getParam("axalta/ccscore/dashboard/CURRENT_PROCESS",s);
    }
    msg.current_process = s;

    int percent = 0;
    if(n.hasParam("axalta/ccscore/dashboard/COMPLETION_PERCENTAGE")){
      n.getParam("axalta/ccscore/dashboard/COMPLETION_PERCENTAGE",percent);
    }
    std::string tape_color_;
    if(n.hasParam("axalta/ccscore/dashboard/TAPE_COLOR")){
      n.getParam("axalta/ccscore/dashboard/TAPE_COLOR",tape_color_);
    }
    
    msg.tape_color = tape_color_;
    msg.completion_percentage = percent;
    msg.air_supply.ready = true;//comes from?
    msg.air_supply.running = true;
    msg.air_supply.fatalerr = false;
    msg.air_supply.remarks = "AirSupply at specified pressure";
    
    msg.paint_gun.ready = true;//comes from ccs lite
    msg.paint_gun.running = true;
    msg.paint_gun.fatalerr = false;
    msg.paint_gun.remarks = "Paintgun at ARM End Effector";
    msg.mir_battery = std::stoi(mir_battery);
    msg.ccs_battery = main_battery;
    msg.gps_data = gps;
    msg.temperature = temp;
    msg.humidity = hum;
    bool emergency;
    bool emergency_hard;
    n.getParam("axalta/ccscore/dashboard/SOFTWARE_EMERGENCY_STOP",emergency);
    n.getParam("axalta/ccscore/ccs_lite_communicate_EMERGENCY",emergency_hard);
    emergency = emergency || emergency_hard;
    if(emergency){msg.emergency_flag = true;}
    else{msg.emergency_flag = false;}
    //ROS_INFO("%s", msg.data.c_str());
    //ROS_INFO_STREAM(diff1);
    //ROS_WARN_THROTTLE(25,"status_node:Could not load configuration file from <path>.Using defaults.");
    //ROS_ERROR_THROTTLE(55, "status_node:Received unexpected NaN value in transform X.");
    //ROS_FATAL_THROTTLE(75, "status_node:node crashed");
    status_pub.publish(msg);
    
    //ros::Duration(10).sleep();
    
     
    
    //jobid++;
    //n.setParam("JOBID",toString(jobid));
     
    
    ros::spinOnce();

    loop_rate.sleep();
    
  }


  return 0;
}

void restartStatusCheckNode(ros::NodeHandle &n){
  ROS_WARN("restart initiated"); 
  mir_battery = "5.0";
  gps = "";
  main_battery = 5.0; 
  temp = "0.0"; 
  hum = "0.0 %"; 
  mir_state = "";
  n.setParam("axalta/ccscore/dashboard/CURRENT_PROCESS","");
  n.setParam("axalta/ccscore/dashboard/COMPLETION_PERCENTAGE",0);
  
  //ros::Duration(0.5).sleep();
  bool u = true;
  while(u){ 
    n.getParam("axalta/ccscore/dashboard/RESTART_CCSCORE_TRIGGER",u);
  }
  ROS_WARN("restart ended");
}
