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

bool data_flag = false;
bool restart_flag = false;

sensor_msgs::PointCloud2::Ptr top_plane_data = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);
sensor_msgs::PointCloud2::Ptr bottom_plane_data = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);
sensor_msgs::PointCloud2::Ptr not_in_plane_data = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);
sensor_msgs::PointCloud2::Ptr flipped_data = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);

std::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

int counting_all = 0;
std::vector<int> flipped_indices;

float min_dot = 10.0;

void pcdViewer(pcl::PointCloud<PointTypeIO> cloud_points, pcl::PointCloud<pcl::Normal> cloud_normals, std::string viewer_title)
{
  /***** PCL Visualizer  *****/
  pcl::PointCloud<PointTypeIO>::Ptr cloud_points_viz(new pcl::PointCloud<PointTypeIO>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_viz(new pcl::PointCloud<pcl::Normal>);
  cloud_points_viz->points = cloud_points.points;
  cloud_normals_viz->points = cloud_normals.points;
  pcl::visualization::PCLVisualizer viewer(viewer_title);
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

  std::cout << "searchPoint : " << searchPoint << std::endl;
  return searchPoint;
}

void splitPointsAndNormals(pcl::PointCloud<PointTypeFull>::Ptr cloud_with_normals, pcl::PointCloud<PointTypeIO>::Ptr cloud_points_, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_)
{

  std::cout << "splitPointsAndNormals" << std::endl;

  cloud_points_->points.clear();
  cloud_normals_->points.clear();

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
    std::cout << point.x << " " << point.y << " " << point.z << std::endl;
    cloud_points_->points.push_back(point);
    cloud_normals_->points.push_back(normal);
  }

  std::cout << cloud_points_->points.size() << std::endl;
  std::cout << cloud_normals_->points.size() << std::endl;
}

bool recalculateNormals(pcl::PointCloud<PointTypeIO>::Ptr horiontal_plane_, pcl::PointCloud<PointTypeFull>::Ptr h_plane_recalc, std::string plane_type)
{ // plane_type - top,bottom

  if (horiontal_plane_->points.empty())
  {
    ROS_WARN("[recalculateNormals] Input cloud is empty");
    return false;
  }
  pcl::search::KdTree<PointTypeIO>::Ptr search_tree(new pcl::search::KdTree<PointTypeIO>);
  pcl::NormalEstimation<PointTypeIO, PointTypeFull> ne;

  if (plane_type == "top")
  {
    ROS_INFO("setting top viewpoint");
    ne.setViewPoint(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
  }
  else if (plane_type == "bottom")
  {
    ROS_INFO("setting bottom viewpoint");
    ne.setViewPoint(0.0, 0.0, std::numeric_limits<float>::min());
  }
  ne.setInputCloud(horiontal_plane_);
  ne.setSearchMethod(search_tree);
  ne.setRadiusSearch(0.05);
  ne.compute(*h_plane_recalc);
  return true;
}

bool enforceNormalSimilarity(const PointTypeFull &point_a, const PointTypeFull &point_b)
{

  Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.getNormalVector3fMap(), point_b_normal = point_b.getNormalVector3fMap();
  if (min_dot > (point_a_normal.dot(point_b_normal)))
  {
    min_dot = point_a_normal.dot(point_b_normal);
    // std::cout << "min_dot : "<<min_dot << std::endl;
  }

  if (point_a_normal.dot(point_b_normal) > std::cos(60.0f / 180.0f * static_cast<float>(M_PI)))
    // std::cout<<"true condition : "<<point_a.normal_x<<"  "<<point_a.normal_y<<"  "<<point_a.normal_z<<"  "<<point_b.normal_x<<"  "<<point_b.normal_y<<"  "<<point_b.normal_z<<"  "<<point_a_normal.dot(point_b_normal)<<std::endl;
    return (true);
  // std::cout<<"false condition : "<<point_a.normal_x<<"  "<<point_a.normal_y<<"  "<<point_a.normal_z<<"  "<<point_b.normal_x<<"  "<<point_b.normal_y<<"  "<<point_b.normal_z<<"  "<<point_a_normal.dot(point_b_normal)<<std::endl;
  return (false);
}

bool checkForClusters(pcl::PointCloud<PointTypeFull>::Ptr cloud_with_normals, pcl::PointCloud<PointTypeFull>::Ptr top_plane, pcl::PointCloud<PointTypeFull>::Ptr bottom_plane, pcl::PointNormal searchPoint, bool &first_search)
{
  std::cout << "cloud_with_normals->size() : " << cloud_with_normals->size() << std::endl;
  for (std::size_t itval = 0; itval < cloud_with_normals->size(); itval++)
  {

    if (!(enforceNormalSimilarity(searchPoint, cloud_with_normals->points[itval])))
    {
      // std::cout<<"more than 70 degree"<<std::endl;
      float normal_x_r = cloud_with_normals->points[itval].normal_x;
      float normal_y_r = cloud_with_normals->points[itval].normal_y;
      float normal_z_r = cloud_with_normals->points[itval].normal_z;

      if (normal_z_r > 0.7)
      {
        std::cout << "top_plane : " << normal_x_r << "  " << normal_y_r << "  " << normal_z_r << std::endl;
        top_plane->points.push_back(cloud_with_normals->points[itval]);
      }
      else if (normal_z_r < -0.7)
      {
        bottom_plane->points.push_back(cloud_with_normals->points[itval]);
        std::cout << "bottom_plane : " << normal_x_r << "  " << normal_y_r << "  " << normal_z_r << std::endl;
      }
      else
      {
        continue;
      }

      counting_all = counting_all + 1;

      flipped_indices.push_back(itval);
    }
  }

  return true;
}

void normalCorrectedCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &point_cloud_data)
{
  if (!data_flag)
  {
    ROS_INFO("PCL conversion started..");
    pcl::PCLPointCloud2 pcl_pc2;
    ROS_INFO("Input Width = [%d]", point_cloud_data->width);
    ROS_INFO("Input Height = [%d]", point_cloud_data->height);
    pcl_conversions::toPCL(*point_cloud_data, pcl_pc2);

    ROS_INFO("commencing initialization");
    // Data containers used
    pcl::PointCloud<PointTypeFull>::Ptr cloud_in(new pcl::PointCloud<PointTypeFull>);

    pcl::PointCloud<PointTypeFull>::Ptr cloud_with_normals(new pcl::PointCloud<PointTypeFull>), cloud_with_normals_nan_removed(new pcl::PointCloud<PointTypeFull>), cloud_with_normals_before_applying_flip(new pcl::PointCloud<PointTypeFull>), top_plane(new pcl::PointCloud<PointTypeFull>), bottom_plane(new pcl::PointCloud<PointTypeFull>);
    pcl::PointCloud<PointTypeRGB>::Ptr flipped_cloud(new pcl::PointCloud<PointTypeRGB>);
    pcl::IndicesClustersPtr clusters(new pcl::IndicesClusters), small_clusters(new pcl::IndicesClusters), large_clusters(new pcl::IndicesClusters);
    pcl::search::KdTree<PointTypeIO>::Ptr search_tree(new pcl::search::KdTree<PointTypeIO>);
    // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    ROS_INFO("initialization done ");
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud_in);

    pcl::PointCloud<PointTypeIO>::Ptr cloud_points_raw(new pcl::PointCloud<PointTypeIO>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_raw(new pcl::PointCloud<pcl::Normal>);

    splitPointsAndNormals(cloud_in, cloud_points_raw, cloud_normals_raw);

    pcdViewer(*cloud_points_raw, *cloud_normals_raw, "input cloud and its normals");

    pcl::PointNormal searchPoint;

    searchPoint = computeCentroid(cloud_in);

    ROS_INFO("Initializing K-d Tree");
    pcl::KdTreeFLANN<pcl::PointNormal> kdtree;

    kdtree.setInputCloud(cloud_in);
    ROS_INFO("init  K-d Tree  Done");

    int iter = 0, total_size = cloud_in->points.size();
    int K = 10;

    ROS_INFO("commencing visited list init");
    std::vector<bool> index_visited(total_size, false);
    bool first_search = true;

    ROS_INFO("calling checkForClusters");
    pcl::copyPointCloud(*cloud_in, *cloud_with_normals_before_applying_flip);
    bool done = checkForClusters(cloud_in, top_plane, bottom_plane, searchPoint, first_search);

    // std::cout<<clusters<<std::endl;
    pcl::PointCloud<PointTypeIO>::Ptr top_cloud_points(new pcl::PointCloud<PointTypeIO>);
    pcl::PointCloud<pcl::Normal>::Ptr top_cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<PointTypeIO>::Ptr bottom_cloud_points(new pcl::PointCloud<PointTypeIO>);
    pcl::PointCloud<pcl::Normal>::Ptr bottom_cloud_normals(new pcl::PointCloud<pcl::Normal>);

    int size_cld = cloud_in->points.size();
    std::cout << "size_cld : " << size_cld << std::endl;

    std::cout << "top_plane : " << std::endl;
    splitPointsAndNormals(top_plane, top_cloud_points, top_cloud_normals);

    std::cout << "bottom_plane : " << std::endl;
    splitPointsAndNormals(bottom_plane, bottom_cloud_points, bottom_cloud_normals);

    pcl::PointCloud<PointTypeFull>::Ptr top_plane_recalc(new pcl::PointCloud<PointTypeFull>);
    pcl::PointCloud<PointTypeFull>::Ptr bottom_plane_recalc(new pcl::PointCloud<PointTypeFull>);

    recalculateNormals(top_cloud_points, top_plane_recalc, "top");
    recalculateNormals(bottom_cloud_points, bottom_plane_recalc, "bottom");

    std::cout << "top_plane_recalc->points.size() : " << top_plane_recalc->points.size() << std::endl;
    splitPointsAndNormals(top_plane_recalc, top_cloud_points, top_cloud_normals);
    splitPointsAndNormals(bottom_plane_recalc, bottom_cloud_points, bottom_cloud_normals);

    int size_cld_p = top_cloud_points->points.size();
    std::cout << "top_cloud_points : " << size_cld_p << std::endl;
    int size_cld_n = top_cloud_normals->points.size();
    std::cout << "top_cloud_normals : " << size_cld_n << std::endl;

    pcdViewer(*top_cloud_points, *top_cloud_normals, "top plane points and normals");
    pcdViewer(*bottom_cloud_points, *bottom_cloud_normals, "bottom plane points and normals");

    std::cout << "[checkForClusters] counting_all : " << counting_all << "  " << cloud_in->points.size() << std::endl;

    for (int j = 0; j < flipped_indices.size(); j++)
    {
      pcl::PointNormal point;
      PointTypeRGB pointrgb;
      point = cloud_in->points[flipped_indices[j]];
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

    pcl::PCLPointCloud2 pcl_pc2_top;
    pcl::toPCLPointCloud2(*top_plane, pcl_pc2_top);
    ROS_INFO("Final processed data");
    ROS_INFO("Width = [%d]", pcl_pc2_top.width);
    ROS_INFO("Height = [%d]", pcl_pc2_top.height);
    ROS_INFO("Total points = [%d]", pcl_pc2_top.width * pcl_pc2_top.height);

    pcl_conversions::fromPCL(pcl_pc2_top, *top_plane_data);
    ROS_INFO("Final converted data");
    ROS_INFO("Width = [%d]", top_plane_data->width);
    ROS_INFO("Height = [%d]", top_plane_data->height);
    top_plane_data->header.frame_id = point_cloud_data->header.frame_id;

    pcl::PCLPointCloud2 pcl_pc2_bottom;
    pcl::toPCLPointCloud2(*bottom_plane, pcl_pc2_bottom);
    ROS_INFO("Final processed data");
    ROS_INFO("Width = [%d]", pcl_pc2_bottom.width);
    ROS_INFO("Height = [%d]", pcl_pc2_bottom.height);
    ROS_INFO("Total points = [%d]", pcl_pc2_bottom.width * pcl_pc2_bottom.height);

    pcl_conversions::fromPCL(pcl_pc2_bottom, *bottom_plane_data);
    ROS_INFO("Final converted data");
    ROS_INFO("Width = [%d]", bottom_plane_data->width);
    ROS_INFO("Height = [%d]", bottom_plane_data->height);
    bottom_plane_data->header.frame_id = point_cloud_data->header.frame_id;

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
    not_in_plane_data->header.frame_id = point_cloud_data->header.frame_id;

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
    flipped_data->header.frame_id = point_cloud_data->header.frame_id;

    data_flag = true;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "horizontal_plane_segmentation");

  ros::NodeHandle n;

  ros::Publisher top_plane_pub = n.advertise<sensor_msgs::PointCloud2>("top_plane_data", 10);
  ros::Publisher input_pub = n.advertise<sensor_msgs::PointCloud2>("input", 10);
  ros::Publisher bottom_plane_pub = n.advertise<sensor_msgs::PointCloud2>("bottom_plane_data", 10);

  ros::Subscriber realsense_sub = n.subscribe("/normal_corrected_cloud", 10, normalCorrectedCloudCallback);

  n.setParam("axalta/ccscore/dashboard/RESTART_FILTERNODE_TRIGGER", false);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    // Check for restart request
    if (n.hasParam("axalta/ccscore/dashboard/RESTART_FILTERNODE_TRIGGER") && n.getParam("axalta/ccscore/dashboard/RESTART_FILTERNODE_TRIGGER", restart_flag))
    {
      if (restart_flag)
      {
        ROS_INFO("Received restart request. Restarting Pointcloud filter Node...");
      }
    }
    if (data_flag && (!restart_flag))
    {
      top_plane_pub.publish(*top_plane_data);
      bottom_plane_pub.publish(*bottom_plane_data);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
