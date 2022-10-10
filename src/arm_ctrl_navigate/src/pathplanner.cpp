#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>  

void get_waypoint_data(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void get_panel_datapoints(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "pathplanner");

  ros::NodeHandle n;
  double arr[20] = {10.0,20.0,30.0,40.0,50.0,60.0,70.0,80.0,90.0,10.0,20.0,30.0,40.0,50.0,60.0,70.0,80.0,90.0,100.0,110.0};
  int len = sizeof(arr)/sizeof(arr[0]);
  ros::Subscriber sub = n.subscribe("waypoint_data", 1000, get_waypoint_data);
  ros::Subscriber sub1 = n.subscribe("panel_datapoints", 1000, get_panel_datapoints);
  ros::Publisher chatter_pub = n.advertise<nav_msgs::Path>("arm_planned_path", 1000);
  
  ros::Rate loop_rate(1);
  int count = 0;
  
  nav_msgs::Path path;
  geometry_msgs::PoseStamped this_pose_stamped;
  
  ROS_INFO("I heard: [%d]", len);
  ROS_INFO("I heard: [%d]", count);
while (ros::ok())
  {
  for(int i=0;i<=(len-1);i++){
    //ROS_INFO("I heard: im in");
    ros::Time current_time = ros::Time::now();
    this_pose_stamped.pose.position.x = arr[i]; 
    this_pose_stamped.pose.position.y = arr[i];
    this_pose_stamped.pose.position.z = arr[i]; 
    this_pose_stamped.pose.orientation.x = 0.0; 
    this_pose_stamped.pose.orientation.y = 0.0; 
    this_pose_stamped.pose.orientation.z = 0.0; 
    this_pose_stamped.pose.orientation.w = 0.0;
    this_pose_stamped.header.stamp=current_time; 
    this_pose_stamped.header.frame_id="lidar-centric"; 
    path.poses.push_back(this_pose_stamped); 
    
    
    
  }
  
  while (chatter_pub.getNumSubscribers() < 1) {}
 
    
    chatter_pub.publish(path);

    ros::spinOnce();
 

    }
    return 0;
}
