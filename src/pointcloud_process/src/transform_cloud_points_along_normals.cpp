#include "ros/ros.h"
#include "std_msgs/String.h"
#include "status_check/stats.h"
#include <sstream>
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

sensor_msgs::PointCloud2::Ptr transformed_cloud_sealer_coat = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);
sensor_msgs::PointCloud2::Ptr transformed_cloud_base_coat1 = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);
sensor_msgs::PointCloud2::Ptr transformed_cloud_base_coat2 = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);
sensor_msgs::PointCloud2::Ptr transformed_cloud_clear_coat1 = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);
sensor_msgs::PointCloud2::Ptr transformed_cloud_clear_coat2 = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);
sensor_msgs::PointCloud2::Ptr base_and_normals = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);

bool dataFlag = false;

float painting_dist_sealer_coat = 0.1524;    // 6 inches
float painting_dist_base_one_coat = 0.1524;  // 6 inches
float painting_dist_base_two_coat = 0.2286;   // 10 inches
float painting_dist_clear_one_coat = 0.1524; // 6 inches
float painting_dist_clear_two_coat = 0.2032; // 8 inches

float coat_sealercoat_clearance = 0, coat_basecoat1_clearance = 0, coat_basecoat2_clearance = 0, coat_clearcoat1_clearance = 0, coat_clearcoat2_clearance = 0; // comes from UI settings
bool restart_flag = false;

void getNodeParams(ros::NodeHandle &n);
void restartNode(ros::NodeHandle &n);
pcl::PointXYZ computeShiftedPoint(float xv, float yv, float zv, float xp, float yp, float zp, float distance, std::string coat_name);

ros::ServiceClient client_gl;

void filteredDataCallback(const sensor_msgs::PointCloud2::ConstPtr &pointCloudData)
{
  try
  {
    if (!dataFlag)
    {
      if (pointCloudData->data.size() > 0)
      {

        ROS_INFO("Surface transformation started..");
        ROS_INFO("Before Moving Least Squares (MLS) surface reconstruction");
        ROS_INFO("Width = [%d]", pointCloudData->width);
        ROS_INFO("Height = [%d]", pointCloudData->height);
        ROS_INFO("Total points = [%d]", pointCloudData->width * pointCloudData->height);
        ROS_INFO("Header [%s]", pointCloudData->header.frame_id);

        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*pointCloudData, pcl_pc2);
        pcl::PointCloud<pcl::PointNormal>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointNormal>);
        pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);

        ROS_INFO("Removing NaN values from the received pointcloud..");
        pcl::PointCloud<pcl::PointNormal>::Ptr outputCloud(new pcl::PointCloud<pcl::PointNormal>);
        std::vector<int> indices;
        temp_cloud->is_dense = false;
        pcl::removeNaNFromPointCloud(*temp_cloud, *outputCloud, indices);
        outputCloud->is_dense = true;

        // Output datasets
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_nan_removed(new pcl::PointCloud<pcl::Normal>);

        float xp = 0, yp = 0, zp = 0;
        float xn = 0, yn = 0, zn = 0;

        pcl::PointCloud<pcl::PointNormal>::Ptr base_points_and_normals(new pcl::PointCloud<pcl::PointNormal>);
        pcl::PointCloud<pcl::PointNormal>::Ptr transformed_pts_sealercoat(new pcl::PointCloud<pcl::PointNormal>);
        pcl::PointCloud<pcl::PointNormal>::Ptr transformed_pts_basecoat1(new pcl::PointCloud<pcl::PointNormal>);
        pcl::PointCloud<pcl::PointNormal>::Ptr transformed_pts_basecoat2(new pcl::PointCloud<pcl::PointNormal>);
        pcl::PointCloud<pcl::PointNormal>::Ptr transformed_pts_clearcoat1(new pcl::PointCloud<pcl::PointNormal>);
        pcl::PointCloud<pcl::PointNormal>::Ptr transformed_pts_clearcoat2(new pcl::PointCloud<pcl::PointNormal>);

        for (int i = 0; i < outputCloud->points.size(); i++)
        {
          pcl::PointNormal transformed_point_sealercoat;
          pcl::PointNormal transformed_point_basecoat1;
          pcl::PointNormal transformed_point_basecoat2;
          pcl::PointNormal transformed_point_clearcoat1;
          pcl::PointNormal transformed_point_clearcoat2;
          pcl::PointNormal points_and_normals;

          // Get the points in pointcloud
          // xp = outputCloud->points[i].x;
          xp = outputCloud->points[i].x;
          // yp = outputCloud->points[i].y;
          yp = outputCloud->points[i].y;
          // zp = outputCloud->points[i].z;
          zp = outputCloud->points[i].z;

          // Get the points in normal
          xn = outputCloud->points[i].normal_x;
          // xn = cloud_normals->points[i].normal_y;
          yn = outputCloud->points[i].normal_y;
          // yn = cloud_normals->points[i].normal_x;
          zn = outputCloud->points[i].normal_z;

          if (!isnan(xn) && !isnan(yn) && !isnan(zn) && !isnan(xp) && !isnan(yp) && !isnan(zp))
          {
            double xvec = xn;
            double yvec = yn;
            double zvec = zn;
            double norm_factor = pow((pow(xvec, 2) + pow(yvec, 2) + pow(zvec, 2)), 0.5);
            double unit_vec_x = xvec / norm_factor;
            double unit_vec_y = yvec / norm_factor;
            double unit_vec_z = zvec / norm_factor;

            transformed_point_sealercoat.normal_x = unit_vec_x;
            transformed_point_basecoat1.normal_x = unit_vec_x;
            transformed_point_basecoat2.normal_x = unit_vec_x;
            transformed_point_clearcoat1.normal_x = unit_vec_x;
            transformed_point_clearcoat2.normal_x = unit_vec_x;
            points_and_normals.normal_x = unit_vec_y;

            transformed_point_sealercoat.normal_y = unit_vec_y;
            transformed_point_basecoat1.normal_y = unit_vec_y;
            transformed_point_basecoat2.normal_y = unit_vec_y;
            transformed_point_clearcoat1.normal_y = unit_vec_y;
            transformed_point_clearcoat2.normal_y = unit_vec_y;
            points_and_normals.normal_y = unit_vec_y;

            transformed_point_sealercoat.normal_z = unit_vec_z;
            transformed_point_basecoat1.normal_z = unit_vec_z;
            transformed_point_basecoat2.normal_z = unit_vec_z;
            transformed_point_clearcoat1.normal_z = unit_vec_z;
            transformed_point_clearcoat2.normal_z = unit_vec_z;
            points_and_normals.normal_z = unit_vec_z;

            pcl::PointXYZ sealerCoat_ShiftedPoint = computeShiftedPoint(xvec, yvec, zvec, xp, yp, zp, painting_dist_sealer_coat, "sealer");
            pcl::PointXYZ baseCoat_1_ShiftedPoint = computeShiftedPoint(xvec, yvec, zvec, xp, yp, zp, painting_dist_base_one_coat, "base1");
            pcl::PointXYZ baseCoat_2_ShiftedPoint = computeShiftedPoint(xvec, yvec, zvec, xp, yp, zp, painting_dist_base_two_coat, "base2");
            pcl::PointXYZ clearCoat_1_ShiftedPoint = computeShiftedPoint(xvec, yvec, zvec, xp, yp, zp, painting_dist_clear_one_coat, "clear1");
            pcl::PointXYZ clearCoat_2_ShiftedPoint = computeShiftedPoint(xvec, yvec, zvec, xp, yp, zp, painting_dist_clear_two_coat, "clear2");

            transformed_point_sealercoat.x = sealerCoat_ShiftedPoint.x;
            transformed_point_basecoat1.x = baseCoat_1_ShiftedPoint.x;
            transformed_point_basecoat2.x = baseCoat_2_ShiftedPoint.x;
            transformed_point_clearcoat1.x = clearCoat_1_ShiftedPoint.x;
            transformed_point_clearcoat2.x = clearCoat_2_ShiftedPoint.x;
            points_and_normals.x = xp;

            transformed_point_sealercoat.y = sealerCoat_ShiftedPoint.y;
            transformed_point_basecoat1.y = baseCoat_1_ShiftedPoint.y;
            transformed_point_basecoat2.y = baseCoat_2_ShiftedPoint.y;
            transformed_point_clearcoat1.y = clearCoat_1_ShiftedPoint.y;
            transformed_point_clearcoat2.y = clearCoat_2_ShiftedPoint.y;
            points_and_normals.y = yp;

            transformed_point_sealercoat.z = sealerCoat_ShiftedPoint.z;
            transformed_point_basecoat1.z = baseCoat_1_ShiftedPoint.z;
            transformed_point_basecoat2.z = baseCoat_2_ShiftedPoint.z;
            transformed_point_clearcoat1.z = clearCoat_1_ShiftedPoint.z;
            transformed_point_clearcoat2.z = clearCoat_2_ShiftedPoint.z;
            points_and_normals.z = zp;

            base_points_and_normals->points.push_back(points_and_normals);
            transformed_pts_sealercoat->points.push_back(transformed_point_sealercoat);
            transformed_pts_basecoat1->points.push_back(transformed_point_basecoat1);
            transformed_pts_basecoat2->points.push_back(transformed_point_basecoat2);
            transformed_pts_clearcoat1->points.push_back(transformed_point_clearcoat1);
            transformed_pts_clearcoat2->points.push_back(transformed_point_clearcoat2);
          }
        }

        // Convert reconstructed cloud after mls to ROS data type

        pcl::PCLPointCloud2 pcl_pc2_o;
        pcl::toPCLPointCloud2(*transformed_pts_sealercoat, pcl_pc2_o);
        pcl_conversions::fromPCL(pcl_pc2_o, *transformed_cloud_sealer_coat);

        pcl::PCLPointCloud2 pcl_pc2_basecoat1_orient;
        pcl::toPCLPointCloud2(*transformed_pts_basecoat1, pcl_pc2_basecoat1_orient);
        pcl_conversions::fromPCL(pcl_pc2_basecoat1_orient, *transformed_cloud_base_coat1);

        pcl::PCLPointCloud2 pcl_pc2_clearcoat1_orient;
        pcl::toPCLPointCloud2(*transformed_pts_clearcoat1, pcl_pc2_clearcoat1_orient);
        pcl_conversions::fromPCL(pcl_pc2_clearcoat1_orient, *transformed_cloud_clear_coat1);

        pcl::PCLPointCloud2 pcl_pc2_basecoat2_orient;
        pcl::toPCLPointCloud2(*transformed_pts_basecoat2, pcl_pc2_basecoat2_orient);
        pcl_conversions::fromPCL(pcl_pc2_basecoat2_orient, *transformed_cloud_base_coat2);

        pcl::PCLPointCloud2 pcl_pc2_clearcoat2_orient;
        pcl::toPCLPointCloud2(*transformed_pts_clearcoat2, pcl_pc2_clearcoat2_orient);
        pcl_conversions::fromPCL(pcl_pc2_clearcoat2_orient, *transformed_cloud_clear_coat2);

        pcl::PCLPointCloud2 pointnormals;
        pcl::toPCLPointCloud2(*base_points_and_normals, pointnormals);
        pcl_conversions::fromPCL(pointnormals, *base_and_normals);

        ROS_INFO("Final output: for Sealer orientation points");
        ROS_INFO("Width = [%d]", transformed_cloud_sealer_coat->width);
        ROS_INFO("Height = [%d]", transformed_cloud_sealer_coat->height);
        ROS_INFO("Total points = [%d]", transformed_cloud_sealer_coat->width * transformed_cloud_sealer_coat->height);
        ROS_INFO("Header [%s]", transformed_cloud_sealer_coat->header.frame_id);
        transformed_cloud_sealer_coat->header.frame_id = "/mir_link";

        ROS_INFO("Final output: for Base 1 orientation points");
        ROS_INFO("Width = [%d]", transformed_cloud_base_coat1->width);
        ROS_INFO("Height = [%d]", transformed_cloud_base_coat1->height);
        ROS_INFO("Total points = [%d]", transformed_cloud_base_coat1->width * transformed_cloud_base_coat1->height);
        ROS_INFO("Header [%s]", transformed_cloud_base_coat1->header.frame_id);
        transformed_cloud_base_coat1->header.frame_id = "/mir_link";

        ROS_INFO("Final output: for Base 2 orientation points");
        ROS_INFO("Width = [%d]", transformed_cloud_base_coat2->width);
        ROS_INFO("Height = [%d]", transformed_cloud_base_coat2->height);
        ROS_INFO("Total points = [%d]", transformed_cloud_base_coat2->width * transformed_cloud_base_coat2->height);
        ROS_INFO("Header [%s]", transformed_cloud_base_coat2->header.frame_id);
        transformed_cloud_base_coat2->header.frame_id = "/mir_link";

        ROS_INFO("Final output: for Clear 1 orientation points");
        ROS_INFO("Width = [%d]", transformed_cloud_clear_coat1->width);
        ROS_INFO("Height = [%d]", transformed_cloud_clear_coat1->height);
        ROS_INFO("Total points = [%d]", transformed_cloud_clear_coat1->width * transformed_cloud_clear_coat1->height);
        ROS_INFO("Header [%s]", transformed_cloud_clear_coat1->header.frame_id);
        transformed_cloud_clear_coat1->header.frame_id = "/mir_link";

        ROS_INFO("Final output: for Clear 2 points");
        ROS_INFO("Width = [%d]", transformed_cloud_clear_coat2->width);
        ROS_INFO("Height = [%d]", transformed_cloud_clear_coat2->height);
        ROS_INFO("Total points = [%d]", transformed_cloud_clear_coat2->width * transformed_cloud_clear_coat2->height);
        ROS_INFO("Header [%s]", transformed_cloud_clear_coat2->header.frame_id);
        transformed_cloud_clear_coat2->header.frame_id = "/mir_link";

        dataFlag = true;
      }
      else
      {
        ROS_ERROR("No data in callback. Retaking...");
      }
    }
  }
  catch (sensor_msgs::PointCloud2::ConstPtr &pointCloudDataExcep)
  {
    ROS_INFO("Exception received at callback>>> Pointcloud or pointer exception in surface model node.");
  }
  catch (...)
  {
    ROS_INFO("Exception received at callback in surface model node.");
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "transform_cloud_points_along_normals");

  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<pointcloud_process::NormalToQuaternion>("normal_to_quaternion");

  client_gl = client;

  ros::Publisher point_and_normal_pub = n.advertise<sensor_msgs::PointCloud2>("points_and_normals", 10);

  ros::Publisher transformed_point_sealercoat_pub = n.advertise<sensor_msgs::PointCloud2>("transformed_point_sealercoat", 10);

  ros::Publisher transformed_point_basecoat1_pub = n.advertise<sensor_msgs::PointCloud2>("transformed_point_basecoat1", 10);

  ros::Publisher transformed_point_basecoat2_pub = n.advertise<sensor_msgs::PointCloud2>("transformed_point_basecoat2", 10);

  ros::Publisher transformed_point_clearcoat1_pub = n.advertise<sensor_msgs::PointCloud2>("transformed_point_clearcoat1", 10);

  ros::Publisher transformed_point_clearcoat2_pub = n.advertise<sensor_msgs::PointCloud2>("transformed_point_clearcoat2", 10);

  ros::Subscriber lidar_sub = n.subscribe("/normal_corrected_cloud", 10, filteredDataCallback);
  // ros::Subscriber lidar_sub = n.subscribe("/zyx_sliced", 10, filteredDataCallback);
  // ros::Subscriber lidar_sub = n.subscribe("/filtered_data_transformed", 10, filteredDataCallback);

  // ros::Subscriber lidar_sub = n.subscribe("/filtered_data", 10, filteredDataCallback);

  n.setParam("axalta/ccscore/dashboard/restart_transform_cloud_points_along_normals_node_trigger", false);

  getNodeParams(n);

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    // Check for restart request
    if (n.hasParam("axalta/ccscore/dashboard/restart_transform_cloud_points_along_normals_node_trigger") && n.getParam("axalta/ccscore/dashboard/restart_transform_cloud_points_along_normals_node_trigger", restart_flag))
    {
      if (restart_flag)
      {
        std::cout<<"restart flag : "<<restart_flag<<std::endl;
        ROS_INFO("Received restart request. Restarting Surface generation Node...");
        restartNode(n);
      }
    }

    if (dataFlag && (!restart_flag))
    {

      transformed_point_sealercoat_pub.publish(*transformed_cloud_sealer_coat);

      transformed_point_basecoat1_pub.publish(*transformed_cloud_base_coat1);

      transformed_point_basecoat2_pub.publish(*transformed_cloud_base_coat2);

      transformed_point_clearcoat1_pub.publish(*transformed_cloud_clear_coat1);

      transformed_point_clearcoat2_pub.publish(*transformed_cloud_clear_coat2);

      point_and_normal_pub.publish(*base_and_normals);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

pcl::PointXYZ computeShiftedPoint(float xv, float yv, float zv, float xp, float yp, float zp, float distance, std::string coat_name)
{

  pcl::PointXYZ shiftedpoint;
  float paint_gun_distance = 0.21344;
  if(coat_name == "clear1" || coat_name == "clear2"){
    paint_gun_distance = 0.219;
  }
  distance = distance + paint_gun_distance;
  try
  {

    shiftedpoint.x = xv * distance + xp;
    shiftedpoint.y = yv * distance + yp;
    shiftedpoint.z = zv * distance + zp;

    // ROS_INFO("x3 = %f, y3 = %f, z3 = %f", shiftedpoint.x, shiftedpoint.y, shiftedpoint.z);
  }
  catch (float ex)
  {
    ROS_INFO("Float exception received during the execution of computeShiftedPoint function in Surface modelling node");
  }
  catch (...)
  {
    ROS_INFO("Exception received during the execution of computeShiftedPoint function in Surface modelling node");
  }

  return shiftedpoint;
}

void getNodeParams(ros::NodeHandle &n)
{
  try
  {
    if (n.hasParam("axalta/ccscore/dashboard/SPRAYGUN_DISTANCE1_sealercoat"))
    {
      if (n.getParam("axalta/ccscore/dashboard/SPRAYGUN_DISTANCE1_sealercoat", coat_sealercoat_clearance))
      {
        ROS_INFO("axalta/ccscore/dashboard/SPRAYGUN_DISTANCE1_sealercoat = %f", coat_sealercoat_clearance);
        painting_dist_sealer_coat = coat_sealercoat_clearance * 0.0254;
      } // Else taking the globally initiated value as 0.1524m or 6 inches
    }
    if (n.hasParam("axalta/ccscore/dashboard/SPRAYGUN_DISTANCE1_basecoat1"))
    {

      if (n.getParam("axalta/ccscore/dashboard/SPRAYGUN_DISTANCE1_basecoat1", coat_basecoat1_clearance))
      {
        ROS_INFO("axalta/ccscore/dashboard/SPRAYGUN_DISTANCE1_basecoat1 = %f", coat_basecoat1_clearance);
        painting_dist_base_one_coat = coat_basecoat1_clearance * 0.0254;
      } // Else taking the globally initiated value as 0.254m or 10 inches
    }
    if (n.hasParam("axalta/ccscore/dashboard/SPRAYGUN_DISTANCE1_basecoat2"))
    {
      if (n.getParam("axalta/ccscore/dashboard/SPRAYGUN_DISTANCE1_basecoat2", coat_basecoat2_clearance))
      {
        ROS_INFO("axalta/ccscore/dashboard/SPRAYGUN_DISTANCE1_basecoat2 = %f", coat_basecoat2_clearance);
        painting_dist_base_two_coat = coat_basecoat2_clearance * 0.0254;
      } // Else taking the globally initiated value as 0.254m or 10 inches
    }
    if (n.hasParam("axalta/ccscore/dashboard/SPRAYGUN_DISTANCE1_clearcoat1"))
    {
      if (n.getParam("axalta/ccscore/dashboard/SPRAYGUN_DISTANCE1_clearcoat1", coat_clearcoat1_clearance))
      {
        ROS_INFO("axalta/ccscore/dashboard/SPRAYGUN_DISTANCE1_clearcoat1 = %d", coat_clearcoat1_clearance);
        painting_dist_clear_one_coat = coat_clearcoat1_clearance * 0.0254;
      } // Else taking the globally initiated value as 0.2032m or 8 inches
    }
    if (n.hasParam("axalta/ccscore/dashboard/SPRAYGUN_DISTANCE1_clearcoat2"))
    {
      if (n.getParam("axalta/ccscore/dashboard/SPRAYGUN_DISTANCE1_clearcoat2", coat_clearcoat2_clearance))
      {
        ROS_INFO("axalta/ccscore/dashboard/SPRAYGUN_DISTANCE1_clearcoat2 = %d", coat_clearcoat2_clearance);
        painting_dist_clear_two_coat = coat_clearcoat2_clearance * 0.0254;
      } // Else taking the globally initiated value as 0.2032m or 8 inches
    }
  }
  catch (float exc)
  {
    ROS_INFO("Float value exception received parameter reading inside surface model node.");
  }
  catch (...)
  {
    ROS_INFO("Exception received during parameter reading inside surface model node.");
  }
}

void restartNode(ros::NodeHandle &n)
{
  ROS_ERROR("filter transform cloud points node");
  dataFlag = false;
  transformed_cloud_sealer_coat = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);
  transformed_cloud_base_coat1 = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);
  transformed_cloud_base_coat2 = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);
  transformed_cloud_clear_coat1 = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);
  transformed_cloud_clear_coat2 = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);

  base_and_normals = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);
  coat_sealercoat_clearance = 0;
  coat_basecoat1_clearance = 0;
  coat_basecoat2_clearance = 0;
  coat_clearcoat1_clearance = 0;
  coat_clearcoat2_clearance = 0;
  if (n.hasParam("axalta/ccscore/dashboard/restart_transform_cloud_points_along_normals_node_trigger"))
  {
    n.setParam("axalta/ccscore/dashboard/restart_transform_cloud_points_along_normals_node_trigger", false);
    restart_flag = false;
    ros::Duration(0.1).sleep();
    n.setParam("axalta/ccscore/dashboard/restart_path_planning_node_trigger", true);
  }
  getNodeParams(n);
  
}
