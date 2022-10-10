#include <ros/ros.h>
#include <tf/transform_broadcaster.h>



int main(int argc, char** argv){
  ros::init(argc, argv, "path_tf2_broadcaster");
  ros::NodeHandle node;
  float x=0.21,y=0.11,z=0.12,R=0,P=0,Y=0;
  tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  while (node.ok()){
  transform.setOrigin(tf::Vector3(x, y, z));
  q.setRPY(R,P,Y);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "arm_base_centric", "arm_tip_centric"));
  }
  ros::spin();
  return 0;
};

