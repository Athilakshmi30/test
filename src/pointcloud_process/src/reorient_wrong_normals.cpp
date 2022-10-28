#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <thread>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/surface/mls.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/norms.h>

using namespace std::chrono_literals;
typedef pcl::PointXYZ PointTypeIO;
typedef pcl::PointXYZRGB PointTypeRGB;
typedef pcl::PointNormal PointTypeFull;

static int polynomial_order = 2;           // 2
static float mls_search_radius = 0.1;      // 0.7;//0.9;
static float normal_search_radius = 0.095; // 0.05

bool dataFlag = false;
bool restart_flag = false;

sensor_msgs::PointCloud2::Ptr normal_corrected_cloud_data = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);
sensor_msgs::PointCloud2::Ptr not_in_plane_data = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);
sensor_msgs::PointCloud2::Ptr flipped_data = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);

std::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

int counting_all = 0;
std::vector<int> flipped_indices;

void getNodeParams(ros::NodeHandle &n);
void restartNode(ros::NodeHandle &n);

void pcdViewer(pcl::PointCloud<PointTypeIO> cloud_points, pcl::PointCloud<pcl::Normal> cloud_normals)
{
  /***** PCL Visualizer  *****/
  pcl::PointCloud<PointTypeIO>::Ptr cloud_points_viz(new pcl::PointCloud<PointTypeIO>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_viz(new pcl::PointCloud<pcl::Normal>);
  cloud_points_viz->points = cloud_points.points;
  cloud_normals_viz->points = cloud_normals.points;
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.setBackgroundColor(0.0, 0.0, 0.8);
  viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_points_viz, cloud_normals_viz, 1, 0.05);

  while (!viewer.wasStopped())
  {
    viewer.spinOnce();
  }
}

pcl::PointNormal computeCentroid(pcl::PointCloud<PointTypeFull>::Ptr cloud_with_normals)
{
  pcl::PointNormal searchPoint;

  pcl::CentroidPoint<pcl::PointNormal> centroid;
  for (int i = 0; i < cloud_with_normals->points.size(); i++)
  {
    centroid.add(cloud_with_normals->points[i]);
  }
  centroid.get(searchPoint);

  //std::cout << "searchPoint : " << searchPoint << std::endl;
  return searchPoint;
}

void splitPointsAndNormals(pcl::PointCloud<PointTypeFull>::Ptr cloud_with_normals, pcl::PointCloud<PointTypeIO>::Ptr cloud_points_, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_)
{

  int size_cldr = cloud_with_normals->points.size();
  for (int i = 0; i < size_cldr; i++)
  {
    PointTypeIO point;
    pcl::Normal normal;
    point.x = cloud_with_normals->points[i].x;
    point.y = cloud_with_normals->points[i].y;
    point.z = cloud_with_normals->points[i].z;
    normal.normal_x = cloud_with_normals->points[i].normal_x;
    normal.normal_y = cloud_with_normals->points[i].normal_y;
    normal.normal_z = cloud_with_normals->points[i].normal_z;

    cloud_points_->points.push_back(point);
    cloud_normals_->points.push_back(normal);
  }
}

bool enforceNormalSimilarity(const PointTypeFull &point_a, const PointTypeFull &point_b)
{

  Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.getNormalVector3fMap(), point_b_normal = point_b.getNormalVector3fMap();

  if (point_a_normal.dot(point_b_normal) > std::cos(95.0f / 180.0f * static_cast<float>(M_PI)))
  {
    //std::cout << point_a_normal.dot(point_b_normal) << "return true" << std::endl;
    return (true);
  }
  //std::cout << "return false" << std::endl;
  return (false);
}

bool checkForFlippedNormals(pcl::PointCloud<PointTypeFull>::Ptr cloud_with_normals, pcl::KdTreeFLANN<pcl::PointNormal> &kdtree, pcl::PointNormal searchPoint, std::vector<bool> &index_visited, bool &first_search)
{

  int K = 20;
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

  if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
  {
    if (first_search)
    {
      first_search = false;
      ROS_INFO("[checkForFlippedNormals] not first search ");
      for (std::size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
      {

        if (!index_visited[pointIdxNKNSearch[i]])
        {
          searchPoint = cloud_with_normals->points[pointIdxNKNSearch[i]];

          index_visited[pointIdxNKNSearch[i]] = true;
          bool done = checkForFlippedNormals(cloud_with_normals, kdtree, searchPoint, index_visited, first_search);

          break;
        }
      }
    }

    for (std::size_t itval = 0; itval < pointIdxNKNSearch.size(); itval++)
    {

      if (!index_visited[pointIdxNKNSearch[itval]])
      {

        if (!(enforceNormalSimilarity(searchPoint, cloud_with_normals->points[pointIdxNKNSearch[itval]])))
        {

          cloud_with_normals->points[pointIdxNKNSearch[itval]].normal_x = -1 * cloud_with_normals->points[pointIdxNKNSearch[itval]].normal_x;
          cloud_with_normals->points[pointIdxNKNSearch[itval]].normal_y = -1 * cloud_with_normals->points[pointIdxNKNSearch[itval]].normal_y;
          cloud_with_normals->points[pointIdxNKNSearch[itval]].normal_z = -1 * cloud_with_normals->points[pointIdxNKNSearch[itval]].normal_z;
          counting_all = counting_all + 1;

          flipped_indices.push_back(pointIdxNKNSearch[itval]);
        }

        searchPoint = cloud_with_normals->points[pointIdxNKNSearch[itval]];

        index_visited[pointIdxNKNSearch[itval]] = true;

        bool done = checkForFlippedNormals(cloud_with_normals, kdtree, searchPoint, index_visited, first_search);
      }
    }

    return true;
  }

  else
  {
    ROS_INFO("[checkForFlippedNormals] Not able to find near K neighbours");
    return false;
  }
}

void filteredCallback(const sensor_msgs::PointCloud2::ConstPtr &pointCloudData)
{
  if (!dataFlag)
  {
    ROS_INFO("PCL conversion started..");
    pcl::PCLPointCloud2 pcl_pc2;
    ROS_INFO("Input Width = [%d]", pointCloudData->width);
    ROS_INFO("Input Height = [%d]", pointCloudData->height);
    pcl_conversions::toPCL(*pointCloudData, pcl_pc2);

    ROS_INFO("commencing initialization");
    // Data containers used
    pcl::PointCloud<PointTypeIO>::Ptr cloud_in(new pcl::PointCloud<PointTypeIO>);

    pcl::PointCloud<PointTypeFull>::Ptr cloud_with_normals(new pcl::PointCloud<PointTypeFull>), cloud_with_normals_nan_removed(new pcl::PointCloud<PointTypeFull>), cloud_with_normals_before_applying_flip(new pcl::PointCloud<PointTypeFull>);
    pcl::PointCloud<PointTypeRGB>::Ptr flipped_cloud(new pcl::PointCloud<PointTypeRGB>);
    pcl::IndicesClustersPtr clusters(new pcl::IndicesClusters), small_clusters(new pcl::IndicesClusters), large_clusters(new pcl::IndicesClusters);
    pcl::search::KdTree<PointTypeIO>::Ptr search_tree(new pcl::search::KdTree<PointTypeIO>);
    // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    ROS_INFO("initialization done ");
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud_in);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<PointTypeFull> smoothened_cloud;
    pcl::PointCloud<PointTypeIO>::Ptr smoothened_cloud_ptr(new pcl::PointCloud<PointTypeIO>);
    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<PointTypeIO, PointTypeFull> mls;
    mls.setComputeNormals(true);
    mls.setPolynomialFit(false);
    // Set parameters
    mls.setInputCloud(cloud_in);
    mls.setPolynomialOrder(polynomial_order);
    mls.setSearchMethod(search_tree);
    mls.setSearchRadius(mls_search_radius);
    ROS_INFO("MLS parameters set");
    // Reconstruct
    mls.process(smoothened_cloud);

    for (size_t i = 0; i < smoothened_cloud.points.size(); ++i)
    {
      const pcl::PointNormal &mls_pt = smoothened_cloud.points[i];

      if (!isnan(mls_pt.x) && !isnan(mls_pt.y) && !isnan(mls_pt.z))
      {

        pcl::PointXYZ pt(mls_pt.x, mls_pt.y, mls_pt.z);

        smoothened_cloud_ptr->points.push_back(pt);
      }
    }

    // Set up a Normal Estimation class and merge data in cloud_with_normals
    ROS_INFO("commencing normal estimation");
    pcl::copyPointCloud(*smoothened_cloud_ptr, *cloud_with_normals);

    pcl::NormalEstimation<PointTypeIO, PointTypeFull> ne;
    ne.setInputCloud(smoothened_cloud_ptr);
    ne.setSearchMethod(search_tree);
    ne.setRadiusSearch(normal_search_radius);
    ne.compute(*cloud_with_normals);

    for (size_t i = 0; i < cloud_with_normals->points.size(); ++i)
    {
      pcl::PointNormal pt = cloud_with_normals->points[i];
      if (!isnan(pt.x) && !isnan(pt.y) && !isnan(pt.z) && !isnan(pt.normal_x) && !isnan(pt.normal_y) && !isnan(pt.normal_z))
      {
        // std::cout<<mls_pt<<std::endl;

        cloud_with_normals_nan_removed->points.push_back(pt);
      }
    }
    ROS_INFO("completed normal estimation");

    pcl::PointCloud<PointTypeIO>::Ptr cloud_points_raw(new pcl::PointCloud<PointTypeIO>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_raw(new pcl::PointCloud<pcl::Normal>);

    splitPointsAndNormals(cloud_with_normals_nan_removed, cloud_points_raw, cloud_normals_raw);

    //pcdViewer(*cloud_points_raw, *cloud_normals_raw);

    pcl::PointNormal searchPoint;

    searchPoint = computeCentroid(cloud_with_normals_nan_removed);

    ROS_INFO("Initializing K-d Tree");
    pcl::KdTreeFLANN<pcl::PointNormal> kdtree;

    kdtree.setInputCloud(cloud_with_normals_nan_removed);
    ROS_INFO("init  K-d Tree  Done");

    int iter = 0, total_size = cloud_with_normals_nan_removed->points.size();
    int K = 10;

    ROS_INFO("commencing visited list init");
    std::vector<bool> index_visited(total_size, false);
    bool first_search = true;

    ROS_INFO("calling checkForFlippedNormals");
    pcl::copyPointCloud(*cloud_with_normals_nan_removed, *cloud_with_normals_before_applying_flip);
    bool done = checkForFlippedNormals(cloud_with_normals_nan_removed, kdtree, searchPoint, index_visited, first_search);
    // std::cout<<clusters<<std::endl;
    pcl::PointCloud<PointTypeIO>::Ptr cloud_points(new pcl::PointCloud<PointTypeIO>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    int size_cld = cloud_with_normals_nan_removed->points.size();
    std::cout << "size_cld : " << size_cld << std::endl;

    splitPointsAndNormals(cloud_with_normals_nan_removed, cloud_points, cloud_normals);

    int size_cld_p = cloud_points->points.size();
    std::cout << "size_cld : " << size_cld_p << std::endl;
    int size_cld_n = cloud_normals->points.size();
    std::cout << "size_cld_n : " << size_cld_n << std::endl;

    //pcdViewer(*cloud_points, *cloud_normals);

    std::cout << "[checkForFlippedNormals] counting_all : " << counting_all << "  " << cloud_with_normals_nan_removed->points.size() << std::endl;

    for (int j = 0; j < flipped_indices.size(); j++)
    {
      pcl::PointNormal point;
      PointTypeRGB pointrgb;
      point = cloud_with_normals->points[flipped_indices[j]];
      pointrgb.x = point.x;
      pointrgb.y = point.y;
      pointrgb.z = point.z;
      pointrgb.r = 1;
      pointrgb.g = 1;
      pointrgb.b = 1;

      flipped_cloud->points.push_back(pointrgb);

      std::cout << "flipped_indices[" << j << "]"
                << " : " << flipped_indices[j] << std::endl;
    }

    pcl::PCLPointCloud2 pcl_pc2_;
    pcl::toPCLPointCloud2(*cloud_with_normals_nan_removed, pcl_pc2_);
    ROS_INFO("Final processed data");
    ROS_INFO("Width = [%d]", pcl_pc2_.width);
    ROS_INFO("Height = [%d]", pcl_pc2_.height);
    ROS_INFO("Total points = [%d]", pcl_pc2_.width * pcl_pc2_.height);

    pcl_conversions::fromPCL(pcl_pc2_, *normal_corrected_cloud_data);
    ROS_INFO("Final converted data");
    ROS_INFO("Width = [%d]", normal_corrected_cloud_data->width);
    ROS_INFO("Height = [%d]", normal_corrected_cloud_data->height);
    normal_corrected_cloud_data->header.frame_id = pointCloudData->header.frame_id;

    pcl::PCLPointCloud2 pcl_pc2_in;
    pcl::toPCLPointCloud2(*cloud_with_normals_before_applying_flip, pcl_pc2_in);
    ROS_INFO("Final processed data");
    ROS_INFO("Width = [%d]", pcl_pc2_in.width);
    ROS_INFO("Height = [%d]", pcl_pc2_in.height);
    ROS_INFO("Total points = [%d]", pcl_pc2_in.width * pcl_pc2_in.height);

    pcl_conversions::fromPCL(pcl_pc2_in, *not_in_plane_data);
    ROS_INFO("Final converted data");
    ROS_INFO("Width = [%d]", not_in_plane_data->width);
    ROS_INFO("Height = [%d]", not_in_plane_data->height);
    not_in_plane_data->header.frame_id = pointCloudData->header.frame_id;

    pcl::PCLPointCloud2 pcl_pc2_f;
    pcl::toPCLPointCloud2(*flipped_cloud, pcl_pc2_f);
    ROS_INFO("Final processed data");
    ROS_INFO("Width = [%d]", pcl_pc2_f.width);
    ROS_INFO("Height = [%d]", pcl_pc2_f.height);
    ROS_INFO("Total points = [%d]", pcl_pc2_f.width * pcl_pc2_f.height);

    pcl_conversions::fromPCL(pcl_pc2_f, *flipped_data);
    ROS_INFO("Final converted data");
    ROS_INFO("Width = [%d]", flipped_data->width);
    ROS_INFO("Height = [%d]", flipped_data->height);
    flipped_data->header.frame_id = pointCloudData->header.frame_id;

    dataFlag = true;
  }
}

void getNodeParams(ros::NodeHandle &n)
{
  try
  {
    // Reading mls_polynomial_order
    if (n.hasParam("axalta/ccscore/pointcloud_process/reorient_wrong_normals/mls_polynomial_order"))
    {
      n.getParam("axalta/ccscore/pointcloud_process/reorient_wrong_normals/mls_polynomial_order", polynomial_order);
      ROS_INFO("axalta/ccscore/pointcloud_process/reorient_wrong_normals/mls_polynomial_order = %d", polynomial_order);
    }
    else
    {
      ROS_INFO("parameter axalta/ccscore/pointcloud_process/reorient_wrong_normals/mls_polynomial_order not read from config. Taking predefined value %d", polynomial_order);
    }

    // Reading mls_search_radius
    if (n.hasParam("axalta/ccscore/pointcloud_process/reorient_wrong_normals/mls_search_radius"))
    {
      n.getParam("axalta/ccscore/pointcloud_process/reorient_wrong_normals/mls_search_radius", mls_search_radius);
      ROS_INFO("axalta/ccscore/pointcloud_process/reorient_wrong_normals/mls_search_radius = %f", mls_search_radius);
    }
    else
    {
      ROS_INFO("parameter axalta/ccscore/pointcloud_process/reorient_wrong_normals/mls_search_radius not read from config. Taking predefined value %f", mls_search_radius);
    }

    // Reading normal_search_radius
    if (n.hasParam("axalta/ccscore/pointcloud_process/reorient_wrong_normals/normal_search_radius"))
    {
      n.getParam("axalta/ccscore/pointcloud_process/reorient_wrong_normals/normal_search_radius", normal_search_radius);
      ROS_INFO("axalta/ccscore/pointcloud_process/reorient_wrong_normals/normal_search_radius = %f", normal_search_radius);
    }
    else
    {
      ROS_INFO("parameter axalta/ccscore/pointcloud_process/reorient_wrong_normals/normal_search_radius not read from config. Taking predefined value %f", normal_search_radius);
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
  ROS_ERROR("filter reorient wrong normals node");
  dataFlag = false;
  normal_corrected_cloud_data = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);
  not_in_plane_data = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);
  flipped_data = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);  
  if (n.hasParam("axalta/ccscore/dashboard/restart_reorient_wrong_normals_trigger"))
  {
    n.setParam("axalta/ccscore/dashboard/restart_reorient_wrong_normals_trigger", false);
    restart_flag = false;
    ros::Duration(0.1).sleep();
    n.setParam("axalta/ccscore/dashboard/restart_horizontal_plane_segmentation_node_trigger", true);
    n.setParam("axalta/ccscore/dashboard/restart_transform_cloud_points_along_normals_node_trigger", true);
    
  }
  getNodeParams(n);
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "reorient_wrong_normals");

  ros::NodeHandle n;

  ros::Publisher normal_corrected_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("normal_corrected_cloud", 1);
  ros::Publisher input_pub = n.advertise<sensor_msgs::PointCloud2>("input", 1);
  ros::Publisher flipped_pub = n.advertise<sensor_msgs::PointCloud2>("flipped_points", 1);

  ros::Subscriber realsense_sub = n.subscribe("/filtered_and_transformed", 10, filteredCallback);

  n.setParam("axalta/ccscore/dashboard/restart_reorient_wrong_normals_trigger", false);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    // Check for restart request
    if (n.hasParam("axalta/ccscore/dashboard/restart_reorient_wrong_normals_trigger") && n.getParam("axalta/ccscore/dashboard/restart_reorient_wrong_normals_trigger", restart_flag))
    {
      if (restart_flag)
      {
        ROS_ERROR("Received restart request. Restarting reorient wrong normals Node...");
        restartNode(n);
      }
    }
    if (dataFlag && (!restart_flag))
    {
      normal_corrected_cloud_pub.publish(*normal_corrected_cloud_data);
      input_pub.publish(*not_in_plane_data);
      flipped_pub.publish(*flipped_data);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
