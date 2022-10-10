#include "ros/ros.h"
#include "std_msgs/String.h"
#include "status_check/stats.h"
#include <sstream>
#define _USE_MATH_DEFINES
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <cmath>
#include "pointcloud_process/NormalToQuaternion.h"
#include <iostream>
#include <fstream>
#include "path_planner_utility/path_planner_utility.h"
#include <unistd.h>

unsigned int microseconds = 1000000;

sensor_msgs::PointCloud2::Ptr transformed_cloud_sealer_coat = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);
sensor_msgs::PointCloud2::Ptr transformed_cloud_base_coat1 = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);
sensor_msgs::PointCloud2::Ptr transformed_cloud_base_coat2 = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);
sensor_msgs::PointCloud2::Ptr transformed_cloud_clear_coat1 = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);
sensor_msgs::PointCloud2::Ptr transformed_cloud_clear_coat2 = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);


static bool restart_flag = false;

ros::ServiceClient client_euler_to_quat;
bool sealer_read = false;
bool base1_read = false;
bool base2_read = false;
bool clear1_read = false;
bool clear2_read = false;

arm_ctrl_navigate::PathStamped quat_shifted_sealer_data_points;
arm_ctrl_navigate::Path quat_shifted_sealer_data_points_full;

arm_ctrl_navigate::PathStamped quat_shifted_base1_data_points;
arm_ctrl_navigate::Path quat_shifted_base1_data_points_full;

arm_ctrl_navigate::PathStamped quat_shifted_base2_data_points;
arm_ctrl_navigate::Path quat_shifted_base2_data_points_full;

arm_ctrl_navigate::PathStamped quat_shifted_clear1_data_points;
arm_ctrl_navigate::Path quat_shifted_clear1_data_points_full;

arm_ctrl_navigate::PathStamped quat_shifted_clear2_data_points;
arm_ctrl_navigate::Path quat_shifted_clear2_data_points_full;

void sealerDataCallback(const arm_ctrl_navigate::Path::ConstPtr &path)
{
  // std::cout << *path << std::endl;
  if (!sealer_read)
  {
    ROS_INFO("Sealer data is processing ... ");
    arm_ctrl_navigate::Path data_rows_sealer;

    arm_ctrl_navigate::PathStamped data_points;
    pcl::PointCloud<pcl::PointXYZ>::Ptr sealercoatpoint_pc(new pcl::PointCloud<pcl::PointXYZ>);
    data_rows_sealer = *path;
    // std::ofstream outdata;
    // std::ofstream outdata2;
    // outdata2.open("/home/axalta/axalta_ws/test_normal_logic/grey_pannel_quat.txt");
    // outdata.open("/home/axalta/axalta_ws/test_normal_logic/grey_pannel_unit.txt");

    for (int itr = 0; itr < data_rows_sealer.path.size(); itr++)
    {

      // std::cout << "Inside.." << std::endl;

      for (int i = 0; i < data_rows_sealer.path[itr].path_msg.size(); i++)
      {
        arm_ctrl_navigate::Plannedpath sealercoat_point;
        auto point = data_rows_sealer.path[itr].path_msg[i];
        // std::cout <<" writing file ... " << std::endl;
        float unit_vec_x = point.ox;
        float unit_vec_y = point.oy;
        float unit_vec_z = point.oz;
        float xp = point.x;
        float yp = point.y;
        float zp = point.z;
        int index_info = point.point_flag;
        // std::cout << "For point x,y,z,vx,vy,vz : " << point.x <<" , "<< point.y <<" , "<< point.z <<" , "<< point.ox <<" , "<< point.oy <<" , "<< point.oz << std::endl;
        // outdata << point.x << " , "  << point.y << " , "   << point.z << " , " << point.ox << " , " << point.oy << " , " << point.oz << std::endl;

        float qx = 0.0, qy = 0.0, qz = 0.0, qw = 0.0, qxn = 0.0, qyn = 0.0, qzn = 0.0, qwn = 0.0;

        pointcloud_process::NormalToQuaternion srv;
        srv.request.x_normal = -1 * unit_vec_x; //-1 * unit_vec_x;
        srv.request.y_normal = -1 * unit_vec_y; //-1 * unit_vec_y;
        srv.request.z_normal = -1 * unit_vec_z; //-1 * unit_vec_z;
        srv.request.rotation_angle = 0.0;
          if (client_euler_to_quat.call(srv))
          {
            qx = srv.response.quat[0];
            qy = srv.response.quat[1];
            qz = srv.response.quat[2];
            qw = srv.response.quat[3];
          }
          else
          {
            ROS_ERROR("Failed to call service normal_to_quaternion");
          }


        float normq = sqrt(qw * qw + qx * qx + qy * qy + qz * qz);

        qxn = qx / normq;
        qyn = qy / normq;
        qzn = qz / normq;
        qwn = qw / normq;

        float offset_quaternion_x = 0.0;
        float offset_quaternion_y = 0.0;
        float offset_quaternion_z = 0.0;
        float offset_quaternion_w = 1.0;

        float required_quat_x = 0.0, required_quat_y = 0.0, required_quat_z = 0.0, required_quat_w = 0.0;

        required_quat_w = offset_quaternion_w * qwn - offset_quaternion_x * qxn - offset_quaternion_y * qyn - offset_quaternion_z * qzn;
        required_quat_x = offset_quaternion_w * qxn + offset_quaternion_x * qwn + offset_quaternion_y * qzn - offset_quaternion_z * qyn;
        required_quat_y = offset_quaternion_w * qyn - offset_quaternion_x * qzn + offset_quaternion_y * qwn + offset_quaternion_z * qxn;
        required_quat_z = offset_quaternion_w * qzn + offset_quaternion_x * qyn - offset_quaternion_y * qxn + offset_quaternion_z * qwn;

        // pcl::PointXYZ sealerCoat_shifted_point = Computeshifted_point(xvec, yvec, zvec,xp, yp, zp, paintingDistConstantsealerCoat);
        pcl::PointXYZ sealerCoat_shifted_point;
        sealerCoat_shifted_point.x = xp;
        sealerCoat_shifted_point.y = yp;
        sealerCoat_shifted_point.z = zp;

        sealercoat_point.ox = required_quat_x;
        sealercoat_point.oy = required_quat_y;
        sealercoat_point.oz = required_quat_z;
        sealercoat_point.ow = required_quat_w;
        sealercoat_point.x = xp; // sealerCoat_shifted_point.x;
        sealercoat_point.y = yp; // sealerCoat_shifted_point.y;
        sealercoat_point.z = zp; // sealerCoat_shifted_point.z;
        sealercoat_point.point_flag = index_info;

        // std::cout << sealercoat_point.x << " , "  << sealercoat_point.y << " , "   << sealercoat_point.z << " , " << sealercoat_point.ox << " , " << sealercoat_point.oy << " , " << sealercoat_point.oz << " , " << sealercoat_point.ow << std::endl;
        // outdata2 << sealercoat_point.x << " , "  << sealercoat_point.y << " , "   << sealercoat_point.z << " , " << sealercoat_point.ox << " , " << sealercoat_point.oy << " , " << sealercoat_point.oz << " , " << sealercoat_point.ow << std::endl;

        quat_shifted_sealer_data_points.path_msg.push_back(sealercoat_point);

        sealercoatpoint_pc->points.push_back(sealerCoat_shifted_point);
      }
    }

    // outdata.close();
    // outdata2.close();
    std::cout << " Sealer finised " << std::endl;

    pcl::PCLPointCloud2 pcl_pc2_;
    pcl::toPCLPointCloud2(*sealercoatpoint_pc, pcl_pc2_);
    pcl_conversions::fromPCL(pcl_pc2_, *transformed_cloud_sealer_coat);

    // ROS_INFO("Final output: orientation points");
    // ROS_INFO("Width = [%d]", transformed_cloud_sealer_coat->width);
    // ROS_INFO("Height = [%d]", transformed_cloud_sealer_coat->height);
    // ROS_INFO("Total points = [%d]", transformed_cloud_sealer_coat->width * transformed_cloud_sealer_coat->height);
    transformed_cloud_sealer_coat->header.frame_id = "/mir_link";
    // ROS_INFO("Header [%s]", transformed_cloud_sealer_coat->header.frame_id);

    quat_shifted_sealer_data_points_full.path.push_back(quat_shifted_sealer_data_points);
    sealer_read = true;
  }
}

void base1DataCallback(const arm_ctrl_navigate::Path::ConstPtr &path)
{
  // std::cout << *path << std::endl;
  if (!base1_read && sealer_read)
  {
    ROS_INFO("Base 1 data is processing ...");
    arm_ctrl_navigate::Path data_rows_base1;

    arm_ctrl_navigate::PathStamped data_points;
    pcl::PointCloud<pcl::PointXYZ>::Ptr basecoat1point_pc(new pcl::PointCloud<pcl::PointXYZ>);
    data_rows_base1 = *path;
    // std::ofstream outdata;
    // std::ofstream outdata2;
    // outdata2.open("/home/axalta/axalta_ws/test_normal_logic/grey_pannel_quat.txt");
    // outdata.open("/home/axalta/axalta_ws/test_normal_logic/grey_pannel_unit.txt");
    for (int itr = 0; itr < data_rows_base1.path.size(); itr++)
    {
      // std::cout << "Inside.." << std::endl;
      for (int i = 0; i < data_rows_base1.path[itr].path_msg.size(); i++)
      {
        arm_ctrl_navigate::Plannedpath basecoat1_point;
        auto point = data_rows_base1.path[itr].path_msg[i];
        // std::cout <<" writing file ... " << std::endl;
        float unit_vec_x = point.ox;
        float unit_vec_y = point.oy;
        float unit_vec_z = point.oz;
        float xp = point.x;
        float yp = point.y;
        float zp = point.z;
        int index_info = point.point_flag;
        // std::cout << "For point x,y,z,vx,vy,vz : " << point.x <<" , "<< point.y <<" , "<< point.z <<" , "<< point.ox <<" , "<< point.oy <<" , "<< point.oz << std::endl;
        // outdata << point.x << " , "  << point.y << " , "   << point.z << " , " << point.ox << " , " << point.oy << " , " << point.oz << std::endl;

        float qx = 0.0, qy = 0.0, qz = 0.0, qw = 0.0, qxn = 0.0, qyn = 0.0, qzn = 0.0, qwn = 0.0;

        pointcloud_process::NormalToQuaternion srv;
        srv.request.x_normal = -1 * unit_vec_x; //-1 * unit_vec_x;
        srv.request.y_normal = -1 * unit_vec_y; //-1 * unit_vec_y;
        srv.request.z_normal = -1 * unit_vec_z; //-1 * unit_vec_z;
        srv.request.rotation_angle = 0.0;
          if (client_euler_to_quat.call(srv))
          {
            qx = srv.response.quat[0];
            qy = srv.response.quat[1];
            qz = srv.response.quat[2];
            qw = srv.response.quat[3];
          }
          else
          {
            ROS_ERROR("Failed to call service normal_to_quaternion");
          }


        float normq = sqrt(qw * qw + qx * qx + qy * qy + qz * qz);

        qxn = qx / normq;
        qyn = qy / normq;
        qzn = qz / normq;
        qwn = qw / normq;

        float offset_quaternion_x = 0.0;
        float offset_quaternion_y = 0.0;
        float offset_quaternion_z = 0.0;
        float offset_quaternion_w = 1.0;

        float required_quat_x = 0.0, required_quat_y = 0.0, required_quat_z = 0.0, required_quat_w = 0.0;

        required_quat_w = offset_quaternion_w * qwn - offset_quaternion_x * qxn - offset_quaternion_y * qyn - offset_quaternion_z * qzn;
        required_quat_x = offset_quaternion_w * qxn + offset_quaternion_x * qwn + offset_quaternion_y * qzn - offset_quaternion_z * qyn;
        required_quat_y = offset_quaternion_w * qyn - offset_quaternion_x * qzn + offset_quaternion_y * qwn + offset_quaternion_z * qxn;
        required_quat_z = offset_quaternion_w * qzn + offset_quaternion_x * qyn - offset_quaternion_y * qxn + offset_quaternion_z * qwn;

        // pcl::PointXYZ sealerCoat_shifted_point = Computeshifted_point(xvec, yvec, zvec,xp, yp, zp, paintingDistConstantsealerCoat);
        pcl::PointXYZ baseCoat1_shifted_point;
        baseCoat1_shifted_point.x = xp;
        baseCoat1_shifted_point.y = yp;
        baseCoat1_shifted_point.z = zp;

        basecoat1_point.ox = required_quat_x;
        basecoat1_point.oy = required_quat_y;
        basecoat1_point.oz = required_quat_z;
        basecoat1_point.ow = required_quat_w;
        basecoat1_point.x = xp; // sealerCoat_shifted_point.x;
        basecoat1_point.y = yp; // sealerCoat_shifted_point.y;
        basecoat1_point.z = zp; // sealerCoat_shifted_point.z;
        basecoat1_point.point_flag = index_info;

        // std::cout << sealercoat_point.x << " , "  << sealercoat_point.y << " , "   << sealercoat_point.z << " , " << sealercoat_point.ox << " , " << sealercoat_point.oy << " , " << sealercoat_point.oz << " , " << sealercoat_point.ow << std::endl;
        // outdata2 << sealercoat_point.x << " , "  << sealercoat_point.y << " , "   << sealercoat_point.z << " , " << sealercoat_point.ox << " , " << sealercoat_point.oy << " , " << sealercoat_point.oz << " , " << sealercoat_point.ow << std::endl;

        quat_shifted_base1_data_points.path_msg.push_back(basecoat1_point);

        basecoat1point_pc->points.push_back(baseCoat1_shifted_point);
      }
    }
    // outdata.close();
    // outdata2.close();
    std::cout << " BaseCoat 1 finised " << std::endl;

    pcl::PCLPointCloud2 pcl_pc2_;
    pcl::toPCLPointCloud2(*basecoat1point_pc, pcl_pc2_);
    pcl_conversions::fromPCL(pcl_pc2_, *transformed_cloud_base_coat1);

    // ROS_INFO("Final output: orientation points");
    // ROS_INFO("Width = [%d]", transformed_cloud_sealer_coat->width);
    // ROS_INFO("Height = [%d]", transformed_cloud_sealer_coat->height);
    // ROS_INFO("Total points = [%d]", transformed_cloud_sealer_coat->width * transformed_cloud_sealer_coat->height);
    transformed_cloud_base_coat1->header.frame_id = "/mir_link";
    // ROS_INFO("Header [%s]", transformed_cloud_sealer_coat->header.frame_id);

    quat_shifted_base1_data_points_full.path.push_back(quat_shifted_base1_data_points);
    base1_read = true;
  }
}

void base2DataCallback(const arm_ctrl_navigate::Path::ConstPtr &path)
{
  // std::cout << *path << std::endl;
  if (!base2_read && base1_read && sealer_read)
  {
    ROS_INFO("Base 2 data is processing ...");
    arm_ctrl_navigate::Path data_rows_base2;

    arm_ctrl_navigate::PathStamped data_points;
    pcl::PointCloud<pcl::PointXYZ>::Ptr basecoat2point_pc(new pcl::PointCloud<pcl::PointXYZ>);
    data_rows_base2 = *path;
    // std::ofstream outdata;
    // std::ofstream outdata2;
    // outdata2.open("/home/axalta/axalta_ws/test_normal_logic/grey_pannel_quat.txt");
    // outdata.open("/home/axalta/axalta_ws/test_normal_logic/grey_pannel_unit.txt");
    for (int itr = 0; itr < data_rows_base2.path.size(); itr++)
    {
      // std::cout << "Inside.." << std::endl;
      for (int i = 0; i < data_rows_base2.path[itr].path_msg.size(); i++)
      {
        arm_ctrl_navigate::Plannedpath basecoat2_point;
        auto point = data_rows_base2.path[itr].path_msg[i];
        // std::cout <<" writing file ... " << std::endl;
        float unit_vec_x = point.ox;
        float unit_vec_y = point.oy;
        float unit_vec_z = point.oz;
        float xp = point.x;
        float yp = point.y;
        float zp = point.z;
        int index_info = point.point_flag;
        // std::cout << "For point x,y,z,vx,vy,vz : " << point.x <<" , "<< point.y <<" , "<< point.z <<" , "<< point.ox <<" , "<< point.oy <<" , "<< point.oz << std::endl;
        // outdata << point.x << " , "  << point.y << " , "   << point.z << " , " << point.ox << " , " << point.oy << " , " << point.oz << std::endl;

        float qx = 0.0, qy = 0.0, qz = 0.0, qw = 0.0, qxn = 0.0, qyn = 0.0, qzn = 0.0, qwn = 0.0;

        pointcloud_process::NormalToQuaternion srv;
        srv.request.x_normal = -1 * unit_vec_x; //-1 * unit_vec_x;
        srv.request.y_normal = -1 * unit_vec_y; //-1 * unit_vec_y;
        srv.request.z_normal = -1 * unit_vec_z; //-1 * unit_vec_z;
        srv.request.rotation_angle = 0.0;
          if (client_euler_to_quat.call(srv))
          {
            qx = srv.response.quat[0];
            qy = srv.response.quat[1];
            qz = srv.response.quat[2];
            qw = srv.response.quat[3];
          }
          else
          {
            ROS_ERROR("Failed to call service normal_to_quaternion");
          }

        float normq = sqrt(qw * qw + qx * qx + qy * qy + qz * qz);

        qxn = qx / normq;
        qyn = qy / normq;
        qzn = qz / normq;
        qwn = qw / normq;

        float offset_quaternion_x = 0.0;
        float offset_quaternion_y = 0.0;
        float offset_quaternion_z = 0.0;
        float offset_quaternion_w = 1.0;

        float required_quat_x = 0.0, required_quat_y = 0.0, required_quat_z = 0.0, required_quat_w = 0.0;

        required_quat_w = offset_quaternion_w * qwn - offset_quaternion_x * qxn - offset_quaternion_y * qyn - offset_quaternion_z * qzn;
        required_quat_x = offset_quaternion_w * qxn + offset_quaternion_x * qwn + offset_quaternion_y * qzn - offset_quaternion_z * qyn;
        required_quat_y = offset_quaternion_w * qyn - offset_quaternion_x * qzn + offset_quaternion_y * qwn + offset_quaternion_z * qxn;
        required_quat_z = offset_quaternion_w * qzn + offset_quaternion_x * qyn - offset_quaternion_y * qxn + offset_quaternion_z * qwn;

        // pcl::PointXYZ sealerCoat_shifted_point = Computeshifted_point(xvec, yvec, zvec,xp, yp, zp, paintingDistConstantsealerCoat);
        pcl::PointXYZ baseCoat2_shifted_point;
        baseCoat2_shifted_point.x = xp;
        baseCoat2_shifted_point.y = yp;
        baseCoat2_shifted_point.z = zp;

        basecoat2_point.ox = required_quat_x;
        basecoat2_point.oy = required_quat_y;
        basecoat2_point.oz = required_quat_z;
        basecoat2_point.ow = required_quat_w;
        basecoat2_point.x = xp; // sealerCoat_shifted_point.x;
        basecoat2_point.y = yp; // sealerCoat_shifted_point.y;
        basecoat2_point.z = zp; // sealerCoat_shifted_point.z;
        basecoat2_point.point_flag = index_info;

        // std::cout << sealercoat_point.x << " , "  << sealercoat_point.y << " , "   << sealercoat_point.z << " , " << sealercoat_point.ox << " , " << sealercoat_point.oy << " , " << sealercoat_point.oz << " , " << sealercoat_point.ow << std::endl;
        // outdata2 << sealercoat_point.x << " , "  << sealercoat_point.y << " , "   << sealercoat_point.z << " , " << sealercoat_point.ox << " , " << sealercoat_point.oy << " , " << sealercoat_point.oz << " , " << sealercoat_point.ow << std::endl;

        quat_shifted_base2_data_points.path_msg.push_back(basecoat2_point);

        basecoat2point_pc->points.push_back(baseCoat2_shifted_point);
      }
    }
    // outdata.close();
    // outdata2.close();
    std::cout << " BaseCoat 2 finised " << std::endl;

    pcl::PCLPointCloud2 pcl_pc2_;
    pcl::toPCLPointCloud2(*basecoat2point_pc, pcl_pc2_);
    pcl_conversions::fromPCL(pcl_pc2_, *transformed_cloud_base_coat2);

    // ROS_INFO("Final output: orientation points");
    // ROS_INFO("Width = [%d]", transformed_cloud_sealer_coat->width);
    // ROS_INFO("Height = [%d]", transformed_cloud_sealer_coat->height);
    // ROS_INFO("Total points = [%d]", transformed_cloud_sealer_coat->width * transformed_cloud_sealer_coat->height);
    transformed_cloud_base_coat2->header.frame_id = "/mir_link";
    // ROS_INFO("Header [%s]", transformed_cloud_sealer_coat->header.frame_id);

    quat_shifted_base2_data_points_full.path.push_back(quat_shifted_base2_data_points);
    base2_read = true;
  }
}

void clear1DataCallback(const arm_ctrl_navigate::Path::ConstPtr &path)
{
  // std::cout << *path << std::endl;
  if (!clear1_read && base2_read && base1_read && sealer_read)
  {
    ROS_INFO("Clear 1 data is processing ...");
    arm_ctrl_navigate::Path data_rows_clear1;

    arm_ctrl_navigate::PathStamped data_points;
    pcl::PointCloud<pcl::PointXYZ>::Ptr clearcoat1point_pc(new pcl::PointCloud<pcl::PointXYZ>);
    data_rows_clear1 = *path;

    int test_count = 0;
    for (int itr = 0; itr < data_rows_clear1.path.size(); itr++)
    {
      // std::cout << "Inside.." << std::endl;
      test_count = test_count + 1;
      for (int i = 0; i < data_rows_clear1.path[itr].path_msg.size(); i++)
      {
        arm_ctrl_navigate::Plannedpath clearcoat1_point;
        auto point = data_rows_clear1.path[itr].path_msg[i];
        // std::cout <<" writing file ... " << std::endl;
        float unit_vec_x = point.ox;
        float unit_vec_y = point.oy;
        float unit_vec_z = point.oz;
        float xp = point.x;
        float yp = point.y;
        float zp = point.z;
        int index_info = point.point_flag;
        // std::cout << "For point x,y,z,vx,vy,vz : " << point.x <<" , "<< point.y <<" , "<< point.z <<" , "<< point.ox <<" , "<< point.oy <<" , "<< point.oz << std::endl;
        //std::cout << point.x << " , "  << point.y << " , "   << point.z << " , " << point.ox << " , " << point.oy << " , " << point.oz << " , " << point.point_flag << std::endl;
        //usleep(microseconds);
        float qx = 0.0, qy = 0.0, qz = 0.0, qw = 0.0, qxn = 0.0, qyn = 0.0, qzn = 0.0, qwn = 0.0;

        pointcloud_process::NormalToQuaternion srv;
        srv.request.x_normal = -1 * unit_vec_x; //-1 * unit_vec_x;
        srv.request.y_normal = -1 * unit_vec_y; //-1 * unit_vec_y;
        srv.request.z_normal = -1 * unit_vec_z; //-1 * unit_vec_z;
        srv.request.rotation_angle =  24.899 * M_PI/180.0; // 24.899 degrees
        
        if (client_euler_to_quat.call(srv))
          {
            qx = srv.response.quat[0];
            qy = srv.response.quat[1];
            qz = srv.response.quat[2];
            qw = srv.response.quat[3];
          }
          else
          {
            ROS_ERROR("Failed to call service normal_to_quaternion");
          }
  

        float normq = sqrt(qw * qw + qx * qx + qy * qy + qz * qz);

        qxn = qx / normq;
        qyn = qy / normq;
        qzn = qz / normq;
        qwn = qw / normq;

        float offset_quaternion_x = 0.0;
        float offset_quaternion_y = 0.0;
        float offset_quaternion_z = 0.0;
        float offset_quaternion_w = 1.0;

        float required_quat_x = 0.0, required_quat_y = 0.0, required_quat_z = 0.0, required_quat_w = 0.0;

        required_quat_w = offset_quaternion_w * qwn - offset_quaternion_x * qxn - offset_quaternion_y * qyn - offset_quaternion_z * qzn;
        required_quat_x = offset_quaternion_w * qxn + offset_quaternion_x * qwn + offset_quaternion_y * qzn - offset_quaternion_z * qyn;
        required_quat_y = offset_quaternion_w * qyn - offset_quaternion_x * qzn + offset_quaternion_y * qwn + offset_quaternion_z * qxn;
        required_quat_z = offset_quaternion_w * qzn + offset_quaternion_x * qyn - offset_quaternion_y * qxn + offset_quaternion_z * qwn;

        // pcl::PointXYZ sealerCoat_shifted_point = Computeshifted_point(xvec, yvec, zvec,xp, yp, zp, paintingDistConstantsealerCoat);
        pcl::PointXYZ clearCoat1_shifted_point;
        clearCoat1_shifted_point.x = xp;
        clearCoat1_shifted_point.y = yp;
        clearCoat1_shifted_point.z = zp;

        clearcoat1_point.ox = required_quat_x;
        clearcoat1_point.oy = required_quat_y;
        clearcoat1_point.oz = required_quat_z;
        clearcoat1_point.ow = required_quat_w;
        clearcoat1_point.x = xp; // sealerCoat_shifted_point.x;
        clearcoat1_point.y = yp; // sealerCoat_shifted_point.y;
        clearcoat1_point.z = zp; // sealerCoat_shifted_point.z;
        clearcoat1_point.point_flag = index_info;

        quat_shifted_clear1_data_points.path_msg.push_back(clearcoat1_point);

        clearcoat1point_pc->points.push_back(clearCoat1_shifted_point);
      }
    }
    std::cout << "test_count : " << test_count << std::endl;
    // outdata.close();
    // outdata2.close();
    std::cout << " ClearCoat 1 finised " << std::endl;

    pcl::PCLPointCloud2 pcl_pc2_;
    pcl::toPCLPointCloud2(*clearcoat1point_pc, pcl_pc2_);
    pcl_conversions::fromPCL(pcl_pc2_, *transformed_cloud_clear_coat1);

    transformed_cloud_clear_coat1->header.frame_id = "/mir_link";

    quat_shifted_clear1_data_points_full.path.push_back(quat_shifted_clear1_data_points);
    clear1_read = true;
  }
}

void clear2DataCallback(const arm_ctrl_navigate::Path::ConstPtr &path)
{
  // std::cout << *path << std::endl;
  if (!clear2_read && clear1_read && base2_read && base1_read && sealer_read)
  {
    ROS_INFO("Clear 2 data is processing ...");
    arm_ctrl_navigate::Path data_rows_clear2;

    arm_ctrl_navigate::PathStamped data_points;
    pcl::PointCloud<pcl::PointXYZ>::Ptr clearcoat2point_pc(new pcl::PointCloud<pcl::PointXYZ>);
    data_rows_clear2 = *path;

    for (int itr = 0; itr < data_rows_clear2.path.size(); itr++)
    {
      // std::cout << "Inside.." << std::endl;
      for (int i = 0; i < data_rows_clear2.path[itr].path_msg.size(); i++)
      {
        arm_ctrl_navigate::Plannedpath clearcoat2_point;
        auto point = data_rows_clear2.path[itr].path_msg[i];
        // std::cout <<" writing file ... " << std::endl;
        float unit_vec_x = point.ox;
        float unit_vec_y = point.oy;
        float unit_vec_z = point.oz;
        float xp = point.x;
        float yp = point.y;
        float zp = point.z;
        int index_info = point.point_flag;

        float qx = 0.0, qy = 0.0, qz = 0.0, qw = 0.0, qxn = 0.0, qyn = 0.0, qzn = 0.0, qwn = 0.0;

        pointcloud_process::NormalToQuaternion srv;
        srv.request.x_normal = -1 * unit_vec_x; //-1 * unit_vec_x;
        srv.request.y_normal = -1 * unit_vec_y; //-1 * unit_vec_y;
        srv.request.z_normal = -1 * unit_vec_z; //-1 * unit_vec_z;
        srv.request.rotation_angle =  24.899 * M_PI/180.0; // 24.899 degrees
        if (client_euler_to_quat.call(srv))
        {
          qx = srv.response.quat[0];
          qy = srv.response.quat[1];
          qz = srv.response.quat[2];
          qw = srv.response.quat[3];
        }
        else
        {
          ROS_ERROR("Failed to call service normal_to_quaternion");
        }
 
        std::cout<<qx<<"  "<<qy<<"  "<<qz<<std::endl;   

        float normq = sqrt(qw * qw + qx * qx + qy * qy + qz * qz);

        qxn = qx / normq;
        qyn = qy / normq;
        qzn = qz / normq;
        qwn = qw / normq;

        float offset_quaternion_x = 0.0;
        float offset_quaternion_y = 0.0;
        float offset_quaternion_z = 0.0;
        float offset_quaternion_w = 1.0;

        float required_quat_x = 0.0, required_quat_y = 0.0, required_quat_z = 0.0, required_quat_w = 0.0;

        required_quat_w = offset_quaternion_w * qwn - offset_quaternion_x * qxn - offset_quaternion_y * qyn - offset_quaternion_z * qzn;
        required_quat_x = offset_quaternion_w * qxn + offset_quaternion_x * qwn + offset_quaternion_y * qzn - offset_quaternion_z * qyn;
        required_quat_y = offset_quaternion_w * qyn - offset_quaternion_x * qzn + offset_quaternion_y * qwn + offset_quaternion_z * qxn;
        required_quat_z = offset_quaternion_w * qzn + offset_quaternion_x * qyn - offset_quaternion_y * qxn + offset_quaternion_z * qwn;

        // pcl::PointXYZ sealerCoat_shifted_point = Computeshifted_point(xvec, yvec, zvec,xp, yp, zp, paintingDistConstantsealerCoat);
        pcl::PointXYZ clearCoat2_shifted_point;
        clearCoat2_shifted_point.x = xp;
        clearCoat2_shifted_point.y = yp;
        clearCoat2_shifted_point.z = zp;

        clearcoat2_point.ox = required_quat_x;
        clearcoat2_point.oy = required_quat_y;
        clearcoat2_point.oz = required_quat_z;
        clearcoat2_point.ow = required_quat_w;
        clearcoat2_point.x = xp; // sealerCoat_shifted_point.x;
        clearcoat2_point.y = yp; // sealerCoat_shifted_point.y;
        clearcoat2_point.z = zp; // sealerCoat_shifted_point.z;
        clearcoat2_point.point_flag = index_info;

        quat_shifted_clear2_data_points.path_msg.push_back(clearcoat2_point);

        clearcoat2point_pc->points.push_back(clearCoat2_shifted_point);
      }
    }

    std::cout << " ClearCoat 2 finised " << std::endl;

    pcl::PCLPointCloud2 pcl_pc2_;
    pcl::toPCLPointCloud2(*clearcoat2point_pc, pcl_pc2_);
    pcl_conversions::fromPCL(pcl_pc2_, *transformed_cloud_clear_coat2);

    transformed_cloud_clear_coat2->header.frame_id = "/mir_link";
    // ROS_INFO("Header [%s]", transformed_cloud_sealer_coat->header.frame_id);

    quat_shifted_clear2_data_points_full.path.push_back(quat_shifted_clear2_data_points);
    clear2_read = true;
  }
}

void restartNode(ros::NodeHandle &n)
{

  transformed_cloud_sealer_coat = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);
  transformed_cloud_base_coat1 = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);
  transformed_cloud_base_coat2 = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);
  transformed_cloud_clear_coat1 = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);
  transformed_cloud_clear_coat2 = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);

  sealer_read = false;
  base1_read = false;
  base2_read = false;
  clear1_read = false;
  clear2_read = false;
  if (n.hasParam("axalta/ccscore/dashboard/restart_calculate_end_effector_orientation_node_trigger"))
  {
    n.setParam("axalta/ccscore/dashboard/restart_calculate_end_effector_orientation_node_trigger", false);
    restart_flag = false;
    ros::Duration(0.1).sleep();
    n.setParam("axalta/ccscore/dashboard/restart_trajectory_planning_service_node_trigger", true);
  }
  

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "calculate_end_effector_orientation");

  ros::NodeHandle n;
  n.setParam("axalta/ccscore/dashboard/restart_calculate_end_effector_orientation_node_trigger", false);

  ros::Publisher shifted_point_sealercoat = n.advertise<arm_ctrl_navigate::Path>("shifted_point_sealercoat", 10);
  ros::Publisher shifted_point_sealercoat_pc = n.advertise<sensor_msgs::PointCloud2>("shifted_point_sealercoat_pc", 10);

  ros::Publisher shifted_point_basecoat1 = n.advertise<arm_ctrl_navigate::Path>("shifted_point_basecoat1", 10);
  ros::Publisher shifted_point_basecoat1_pc = n.advertise<sensor_msgs::PointCloud2>("shifted_point_basecoat1_pc", 10);

  ros::Publisher shifted_point_basecoat2 = n.advertise<arm_ctrl_navigate::Path>("shifted_point_basecoat2", 10);
  ros::Publisher shifted_point_basecoat2_pc = n.advertise<sensor_msgs::PointCloud2>("shifted_point_basecoat2_pc", 10);

  ros::Publisher shifted_point_clearcoat1 = n.advertise<arm_ctrl_navigate::Path>("shifted_point_clearcoat1", 10);
  ros::Publisher shifted_point_clearcoat1_pc = n.advertise<sensor_msgs::PointCloud2>("shifted_point_clearcoat1_pc", 10);

  ros::Publisher shifted_point_clearcoat2 = n.advertise<arm_ctrl_navigate::Path>("shifted_point_clearcoat2", 10);
  ros::Publisher shifted_point_clearcoat2_pc = n.advertise<sensor_msgs::PointCloud2>("shifted_point_clearcoat2_pc", 10);

  ros::ServiceClient client = n.serviceClient<pointcloud_process::NormalToQuaternion>("normal_to_quaternion");

  client_euler_to_quat = client;

  ros::Subscriber sealer_sub = n.subscribe("/sealercoat", 5, sealerDataCallback);
  ros::Subscriber base1_sub = n.subscribe("/basecoat1", 5, base1DataCallback);
  ros::Subscriber base2_sub = n.subscribe("/basecoat2", 5, base2DataCallback);
  ros::Subscriber clear1_sub = n.subscribe("/clearcoat1", 5, clear1DataCallback);
  ros::Subscriber clear2_sub = n.subscribe("/clearcoat2", 5, clear2DataCallback);

  ros::Rate loop_rate(100);
  
  ROS_INFO("shifted_points node is up and running");
  while (ros::ok())
  {
    if (n.hasParam("axalta/ccscore/dashboard/restart_calculate_end_effector_orientation_node_trigger") && n.getParam("axalta/ccscore/dashboard/restart_calculate_end_effector_orientation_node_trigger", restart_flag))
    {
      if (restart_flag)
      {
        ROS_INFO("Received restart request. Restarting calculate_end_effector_orientation Node...");
        restartNode(n);
      }
    }

    if (sealer_read)
    {
      shifted_point_sealercoat.publish(quat_shifted_sealer_data_points_full);
      shifted_point_sealercoat_pc.publish(*transformed_cloud_sealer_coat);
    }
    if (base1_read)
    {
      shifted_point_basecoat1.publish(quat_shifted_base1_data_points_full);
      shifted_point_basecoat1_pc.publish(*transformed_cloud_base_coat1);
    }

    if (base2_read)
    {
      shifted_point_basecoat2.publish(quat_shifted_base2_data_points_full);
      shifted_point_basecoat2_pc.publish(*transformed_cloud_base_coat2);
    }

    if (clear1_read)
    {
      shifted_point_clearcoat1.publish(quat_shifted_clear1_data_points_full);
      shifted_point_clearcoat1_pc.publish(*transformed_cloud_clear_coat1);
    }

    if (clear2_read)
    {
      shifted_point_clearcoat2.publish(quat_shifted_clear2_data_points_full);
      shifted_point_clearcoat2_pc.publish(*transformed_cloud_clear_coat2);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}


