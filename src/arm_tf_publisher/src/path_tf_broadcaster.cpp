#include <ros/ros.h>
#include <tf/transform_broadcaster.h>



int main(int argc, char** argv){
  ros::init(argc, argv, "path_tf_broadcaster");
  ros::NodeHandle node;
  float x=0,y=0,z=0.07,R=0,P=0,Y=0;
  tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  ros::Rate rate(500);    
  while (node.ok()){
  transform.setOrigin(tf::Vector3(x, y, z));
  q.setRPY(R,P,Y);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "wrist_3_link", "single_camera_link"));
  rate.sleep();
  }
  ros::spin();
  return 0;
};

