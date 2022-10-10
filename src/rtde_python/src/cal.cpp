#include <tf2/LinearMath/Quaternion.h>
#include <ros/ros.h>


#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include "geometry_msgs/PoseStamped.h"
#include <vector>
#include <tf/tf.h>
int main(int argc, char **argv)
{

  ros::init(argc, argv, "cal");


  ros::NodeHandle n;


  //ros::Publisher chatter_pub = n.advertise<tf::Pose>("chatter", 1000);

  ros::Rate loop_rate(10);


  int count = 0;
  while (ros::ok())
  {

    std_msgs::Header header;
    geometry_msgs::Pose pos1,pos2,msg;
    //std::vector<geometry_msgs::PoseStamped::ConstPtr> pos1,pos2;


    //pose pos1;
    //pose pos2;
    
    tf2::Quaternion pos2_Quaternion;
    pos2_Quaternion.setRPY( -0.13626916220301089, -2.4862022859701076, 1.7553923003422427 );  // Create this quaternion from roll/pitch/yaw (in radians)
    pos1.position.x = -0.100125164015;
    pos1.position.y = 0.698380088359;
    pos1.position.z =  0.85331810385;
    pos1.orientation.x = -0.815880146711;
    pos1.orientation.y = 0.0453901513915;
    pos1.orientation.z = -0.0453883791941;
    pos1.orientation.w = 0.574647035487;


    pos2.position.x = -0.100125164015;
    pos2.position.y = -0.100125164015;
    pos2.position.z = -0.100125164015;
    pos2.orientation.x = pos2_Quaternion.getX();
    pos2.orientation.y = pos2_Quaternion.getY();
    pos2.orientation.z = pos2_Quaternion.getZ();
    pos2.orientation.w = pos2_Quaternion.getW();
      
    tf::Pose tf_pos1,tf_pos2,tf_pos3; 
    tf::poseMsgToTF(pos1,tf_pos1);
    tf::poseMsgToTF(pos2,tf_pos2);
    
    tf::poseTFToMsg(tf_pos3, msg);
    //tf_pos1.inverseTimes(tf_pos2)

    // Print the quaternion components (0,0,0,1)
    //ROS_INFO_STREAM("x: " << pos2_Quaternion.getX() << " y: " << pos2_Quaternion.getY() << 
    //            " z: " << pos2_Quaternion.getZ() << " w: " << pos2_Quaternion.getW());
    tf_pos3 = tf_pos1.inverseTimes(tf_pos2);

    //ROS_INFO_STREAM("x: " << tf_pos3.getX() << " y: " << tf_pos3.getY() << 
    //            " z: " << tf_pos3.getZ() << " w: " << tf_pos3.getW());
    //ROS_INFO_STREAM("val %" , tf_pos3);
    //std::cout << tf_pos3.getBasis().Matrix3x3 << std::endl;
    //std_msgs::String msg;

    //std::stringstream ss;
    //ss << "hello world " << << count;

    //msg.data = ss.str();

    //ROS_INFO("%s", msg.data.c_str());


    //chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
