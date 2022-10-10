#ifndef PATH_PLANNER_UTILITY_H
#define PATH_PLANNER_UTILITY_H


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "status_check/stats.h"
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include "arm_ctrl_navigate/PathStamped.h"
#include "arm_ctrl_navigate/Plannedpath.h"
#include "arm_ctrl_navigate/Path.h"
#include <algorithm>
#include <string>
#include <vector>

#include <iostream>
using std::cerr;
using std::endl;
#include <fstream>
using std::ofstream;
#include <cstdlib>

using namespace std;

#define TOPTOBOTTOM 0
#define BOTTOMTOTOP 1
#define LEFTTORIGHT 0
#define RIGHTTOLEFT 1
#define POINT_LIMIT 5
#define TOP_INDEX_ROW_DIST 0.0762f //0.1524f - 6 inches , 0.1778 - 7 inches
#define BOTTOM_INDEX_ROW_DIST 0.0762f //0.0762f - 3 inches
#define INDEX_PILOT_ROW_DIST 0.0635f //0.05f - 1.96 inches


static float rowThreshold_sealer = 0.0635;       //Distance between rows 2.5 inches
static float rowThreshold_base_coat_1 = 0.0381;  //Distance between rows 1.5 inches
static float rowThreshold_base_coat_2 = 0.0508;  //Distance between rows 2 inches
static float rowThreshold_clear_coat_1 = 0.0762; //Distance between rows 3 inches
static float rowThreshold_clear_coat_2 = 0.0762; //Distance between rows 3 inches
static float rowThreshold_buff = 0.0127;         //Buffer value 0.5 inches will be reduced from all thresholds
static float pointThreshold = 0.2;        //Distance between points 5 cm
static float pointBuffer = 0.005;           //Distance buffer
//static float indexWidth = 0.025;                  //Keeping default value of the index as 2.5 cm
static float indexWidth = 0.1;
static string horizontalSweep = "LTR";           //Horizontal sweep default direction Left to Right
static string verticalSweep = "TTB";              //Vertical sweep default direction Top to Bottom
static int number_of_points_buff = 10;
//static float top_index_row_distance = 0.1524;
static float top_index_row_distance = 0.0635;//0.09
static float bottom_index_row_distance = 0.0635;// 0.0762
static float index_pilot_row_distance = 0.0635;// 0.05


struct ProcessFlow
{
    bool sealer_coat_completed = false;
    bool base_coat_1_completed = false; //newly added
    bool base_coat_2_completed = false;  //newly added
    bool clear_coat_1_completed = false;  //newly added
    bool clear_coat_2_completed = false;  //newly added
    bool sealer_coat_pc_data_read = false;
    bool base_coat_1_pc_data_read = false;  //newly added
    bool base_coat_2_pc_data_read = false;   //newly added
    bool clear_coat_1_pc_data_read = false;  //newly added
    bool clear_coat_2_pc_data_read = false;  //newly added
    bool sealer_coat_orient_data_read = false;
    bool base_coat_1_orient_data_read = false;   //newly added
    bool base_coat_2_orient_data_read = false;   //newly added
    bool clear_coat_1_orient_data_read = false;  //newly added
    bool clear_coat_2_orient_data_read = false;  //newly added
};


struct MinMaxX {
    float minx;
    float maxx;
    
};

float Approximate(float input);
float GetRowDistance(arm_ctrl_navigate::Plannedpath pt1, arm_ctrl_navigate::Plannedpath pt2);
float GetDistance(arm_ctrl_navigate::Plannedpath pt1, arm_ctrl_navigate::Plannedpath pt2);
void LogPointsToFile(arm_ctrl_navigate::Path::Ptr trajectory, std::string file_name);
void LogToFile(std::vector<double> val, std::string file_name);
arm_ctrl_navigate::Path::Ptr SortCloud(arm_ctrl_navigate::Path::Ptr trajectory_in);
pcl::PointCloud<pcl::PointXYZ>::Ptr SortCloudByZ(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
pcl::PointCloud<pcl::PointNormal>::Ptr SortCloudByZ(pcl::PointCloud<pcl::PointNormal>::Ptr cloud);
MinMaxX FindminmaxX(arm_ctrl_navigate::Path::Ptr trajectory);

#endif
