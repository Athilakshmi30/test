#include "ros/ros.h"
#include "std_msgs/String.h"
#include "status_check/stats.h"
#include <sstream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>

sensor_msgs::PointCloud2::Ptr down_sampled_data = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);
sensor_msgs::PointCloud2::Ptr filtered_data = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);
sensor_msgs::PointCloud2::Ptr filtered_data_transformed = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);

static bool data_flag = false;
static bool msg_count = false;
static float min_depth_threshold = 0.6;
static float max_depth_threshold = 1.7;
static float voxel_leaf_size = 0.01f;
static int statfilter_meank = 100;
static float statfilter_stddev = 1.0;
static bool restart_flag = false;

void getNodeParams(ros::NodeHandle &n);
void restartNode(ros::NodeHandle &n);

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &point_cloud_data)
{
  try
  {
    if (!msg_count)
    {
      if (point_cloud_data->data.size() > 0)
      {

        ROS_INFO("PCL conversion started..");
        pcl::PCLPointCloud2 pcl_pc2;
        ROS_INFO("Input Width = [%d]", point_cloud_data->width);
        ROS_INFO("Input Height = [%d]", point_cloud_data->height);
        pcl_conversions::toPCL(*point_cloud_data, pcl_pc2);

        ROS_INFO("Data info before downsampling");
        ROS_INFO("Width = [%d]", pcl_pc2.width);
        ROS_INFO("Height = [%d]", pcl_pc2.height);
        ROS_INFO("Total points = [%d]", pcl_pc2.width * pcl_pc2.height);

        pcl::PCLPointCloud2::Ptr voxel_input(new pcl::PCLPointCloud2());
        pcl::PCLPointCloud2::Ptr voxel_filtered(new pcl::PCLPointCloud2());
        *voxel_input = pcl_pc2;

        // Voxel grid filter -- Downsampling
        //  Create the filtering object
        ROS_INFO("Downsampling started..");
        pcl::VoxelGrid<pcl::PCLPointCloud2> voxFilter;
        voxFilter.setInputCloud(voxel_input);
        voxFilter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
        voxFilter.filter(*voxel_filtered);

        ROS_INFO("Data info before range filtering");
        ROS_INFO("Width = [%d]", voxel_filtered->width);
        ROS_INFO("Height = [%d]", voxel_filtered->height);
        ROS_INFO("Total points = [%d]", voxel_filtered->width * voxel_filtered->height);

        pcl_conversions::fromPCL(*voxel_filtered, *down_sampled_data);
        down_sampled_data->header.frame_id = point_cloud_data->header.frame_id;
        // can publish downsampled output
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        // pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud); //For bypassing downsampling
        pcl::fromPCLPointCloud2(*voxel_filtered, *temp_cloud);
        // pcl::fromPCLPointCloud2(*voxel_input, *temp_cloud);
        float minThreshold2 = min_depth_threshold * min_depth_threshold;
        float maxThreshold2 = max_depth_threshold * max_depth_threshold;

        ROS_INFO("Range filtering started..");

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
        // Perform the range filtering
        for (int p = 0; p < temp_cloud->points.size(); ++p)
        {
          // find the squared distance from the origin.
          float pointDepth2 = ((temp_cloud->points[p].x * temp_cloud->points[p].x) +
                               (temp_cloud->points[p].y * temp_cloud->points[p].y) +
                               (temp_cloud->points[p].z * temp_cloud->points[p].z));

          if (!(pointDepth2 <= minThreshold2 || pointDepth2 >= maxThreshold2) || temp_cloud->points[p].x < 1.1 || temp_cloud->points[p].x > 0.15) // && (z>-0.254))//doubt
          {
            temp_cloud2->points.push_back(temp_cloud->points[p]);
          }
        }

        ROS_INFO("Data info before StatisticalOutlierRemoval");
        ROS_INFO("Total points = [%d]", temp_cloud2->points.size());

        // StatisticalOutlierRemoval filter
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
        // Create the filtering object
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud(temp_cloud2);
        sor.setMeanK(statfilter_meank);

        sor.setStddevMulThresh(statfilter_stddev);
        sor.filter(*cloud_filtered);

        // Convert to ROS data type
        pcl::PCLPointCloud2 pcl_pc2_;

        if (temp_cloud2->points.size() > 50)
        {
          pcl::toPCLPointCloud2(*temp_cloud2, pcl_pc2_);
          ROS_INFO("Final processed data");
          ROS_INFO("Width = [%d]", pcl_pc2_.width);
          ROS_INFO("Height = [%d]", pcl_pc2_.height);
          ROS_INFO("Total points = [%d]", pcl_pc2_.width * pcl_pc2_.height);
          pcl_conversions::fromPCL(pcl_pc2_, *filtered_data);
          ROS_INFO("Final converted data");
          ROS_INFO("Width = [%d]", filtered_data->width);
          ROS_INFO("Height = [%d]", filtered_data->height);
          filtered_data->header.frame_id = point_cloud_data->header.frame_id;

          data_flag = true;
          msg_count = true;
        }
        else
        {
          data_flag = false;
          msg_count = false;
        }
      }
    }
  }
  catch (sensor_msgs::PointCloud2::ConstPtr &point_cloud_data_excep)
  {
    ROS_INFO("Exception received at callback>>> Pointcloud or pointer exception in pointcloud filter node.");
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "point_cloud_filter");

  ros::NodeHandle n;

  ros::Publisher filtered_pub = n.advertise<sensor_msgs::PointCloud2>("filtered_data", 10);

  ros::Publisher downsampled_pub = n.advertise<sensor_msgs::PointCloud2>("downsampled_data", 10);

  ros::Subscriber realsense_sub = n.subscribe("/rgb_pointcloud", 10, pointCloudCallback);

  n.setParam("axalta/ccscore/dashboard/restart_filternode_trigger", false);

  getNodeParams(n);

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    // Check for restart request
    if (n.hasParam("axalta/ccscore/dashboard/restart_filternode_trigger") && n.getParam("axalta/ccscore/dashboard/restart_filternode_trigger", restart_flag))
    {
      if (restart_flag)
      {
        ROS_INFO("Received restart request. Restarting Pointcloud filter Node...");
        restartNode(n);
      }
    }
    if (data_flag && (!restart_flag))
    {
      downsampled_pub.publish(*down_sampled_data);
      filtered_pub.publish(*filtered_data);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

void restartNode(ros::NodeHandle &n)
{
  data_flag = false;
  msg_count = false;
  filtered_data = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);
  if (n.hasParam("axalta/ccscore/dashboard/restart_filternode_trigger"))
  {
    n.setParam("axalta/ccscore/dashboard/restart_filternode_trigger", false);
    restart_flag = false;
    ros::Duration(0.1).sleep();
    n.setParam("axalta/ccscore/dashboard/restart_reorient_wrong_normals_trigger", true);
    ROS_INFO("Triggered restarting Surface Node");
  }
  getNodeParams(n);
}

void getNodeParams(ros::NodeHandle &n)
{

  if (n.hasParam("axalta/ccscore/pointcloud_process/point_cloud_filter/voxel_leaf_size") && n.getParam("axalta/ccscore/pointcloud_process/point_cloud_filter/voxel_leaf_size", voxel_leaf_size))
  {
    ROS_INFO("reading param axalta/ccscore/pointcloud_process/point_cloud_filter/voxel_leaf_size = %f from config file", voxel_leaf_size);
  }
  else
  {
    // Else taking the globally initiated value as 0.008 m
    ROS_INFO("Couldn't read param axalta/ccscore/pointcloud_process/point_cloud_filter/voxel_leaf_size from config file. Taking the value as %f", voxel_leaf_size);
  }

  if (n.hasParam("axalta/ccscore/pointcloud_process/point_cloud_filter/statfilter_meank") && n.getParam("axalta/ccscore/pointcloud_process/point_cloud_filter/statfilter_meank", statfilter_meank))
  {
    ROS_INFO("axalta/ccscore/pointcloud_process/point_cloud_filter/statfilter_meank = %d from config file", statfilter_meank);
  }
  else
  {
    // Else taking the globally initiated value as 180
    ROS_INFO("Couldn't read param axalta/ccscore/pointcloud_process/point_cloud_filter/statfilter_meank from config file. Taking the value as %d", statfilter_meank);
  }

  if (n.hasParam("axalta/ccscore/pointcloud_process/point_cloud_filter/statfilter_stddev") && n.getParam("axalta/ccscore/pointcloud_process/point_cloud_filter/statfilter_stddev", statfilter_stddev))
  {
    ROS_INFO("axalta/ccscore/pointcloud_process/point_cloud_filter/statfilter_stddev = %f from config file", statfilter_stddev);
  }
  else
  {
    // Else taking the globally initiated value as 200
    ROS_INFO("Couldn't read param axalta/ccscore/pointcloud_process/point_cloud_filter/statfilter_stddev from config file. Taking the value as %f", statfilter_stddev);
  }

  if (n.hasParam("axalta/ccscore/pointcloud_process/point_cloud_filter/min_depth") && n.getParam("axalta/ccscore/pointcloud_process/point_cloud_filter/min_depth", min_depth_threshold))
  {
    ROS_INFO("axalta/ccscore/pointcloud_process/point_cloud_filter/min_depth = %f from config file", min_depth_threshold);
  }
  else
  {
    // Else taking the globally initiated value as 0.5 m
    ROS_INFO("Couldn't read param axalta/ccscore/pointcloud_process/point_cloud_filter/min_depth from config file. Taking the value as %f", min_depth_threshold);
  }

  if (n.hasParam("axalta/ccscore/pointcloud_process/point_cloud_filter/max_depth") && n.getParam("axalta/ccscore/pointcloud_process/point_cloud_filter/max_depth", max_depth_threshold))
  {
    ROS_INFO("axalta/ccscore/pointcloud_process/point_cloud_filter/max_depth = %f from config file", max_depth_threshold);
  }
  else
  {
    // Else taking the globally initiated value as 1.73 m
    ROS_INFO("Couldn't read param axalta/ccscore/pointcloud_process/point_cloud_filter/max_depth from config file. Taking the value as %f", max_depth_threshold);
  }
}
