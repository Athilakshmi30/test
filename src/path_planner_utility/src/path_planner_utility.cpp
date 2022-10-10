#include "path_planner_utility/path_planner_utility.h"

double roundd(double var) //detachable
{
    // 37.66666 * 100 =3766.66
    // 3766.66 + .5 =3767.16    for rounding off value
    // then type cast to int so value is 3767
    // then divided by 100 so the value converted into 37.67
    double value = (int)(var * 1000 + .5);
    return (double)value / 1000;
}

float Approximate(float input) //detachable
{
    try
    {
        return (std::ceil(input * 50) / 50);
    }
    catch (float ex)
    {
        ROS_INFO("Float exception received during the execution of Approximate function inside path planner node");
        return 0;
    }
    catch (...)
    {
        ROS_INFO("Exception received during the execution of Approximate function inside path planner node");
        return 0;
    }
}


arm_ctrl_navigate::Path::Ptr SortCloud(arm_ctrl_navigate::Path::Ptr trajectory_in)//detachable
{

    try
    {
        for (int i = 0; i < trajectory_in->path.size(); i++)
        {
            std::sort(trajectory_in->path[i].path_msg.begin(), trajectory_in->path[i].path_msg.end(), [](arm_ctrl_navigate::Plannedpath a, arm_ctrl_navigate::Plannedpath b)
                      { return a.x < b.x; });
        }
    }
    catch (...)
    {
        ROS_INFO("Exception received during the execution of SortCloud function inside path planner node");
    }
    return trajectory_in;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr SortCloudByZ(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)//detachable
{

    try
    {
        for (int i = 0; i < cloud->points.size(); i++)
        {
            std::sort(cloud->points.begin(), cloud->points.end(), [](pcl::PointXYZ a, pcl::PointXYZ b)
                      { return a.z < b.z; });
        }
    }
    catch (...)
    {
        ROS_INFO("Exception received during the execution of SortCloudByZ function inside path planner node");
    }
    return cloud;
}

pcl::PointCloud<pcl::PointNormal>::Ptr SortCloudByZ(pcl::PointCloud<pcl::PointNormal>::Ptr cloud)//detachable
{

    try
    {
        for (int i = 0; i < cloud->points.size(); i++)
        {
            std::sort(cloud->points.begin(), cloud->points.end(), [](pcl::PointNormal a, pcl::PointNormal b)
                      { return a.z < b.z; });
        }
    }
    catch (...)
    {
        ROS_INFO("Exception received during the execution of SortCloudByZ function inside path planner node");
    }
    return cloud;
}


float GetDistance(arm_ctrl_navigate::Plannedpath pt1, arm_ctrl_navigate::Plannedpath pt2)//detachable
{
    try
    {
        return sqrt(pow((pt1.x - pt2.x), 2) + pow((pt1.y - pt2.y), 2) + pow((pt1.z - pt2.z), 2));
    }
    catch (arm_ctrl_navigate::Plannedpath ex)
    {
        ROS_INFO("arm_ctrl_navigate::Plannedpath exception received during the execution of GetDistance function inside path planner node");
        return 0;
    }
    catch (...)
    {
        ROS_INFO("Exception received during the execution of GetDistance function inside path planner node");
        return 0;
    }
}

float GetRowDistance(arm_ctrl_navigate::Plannedpath pt1, arm_ctrl_navigate::Plannedpath pt2)//detachable
{
    try
    {
        return sqrt(pow((pt1.z - pt2.z), 2));
        //return sqrt(pow((pt1.x - pt2.x), 2) + pow((pt1.y - pt2.y), 2) + pow((pt1.z - pt2.z), 2));
    }
    catch (arm_ctrl_navigate::Plannedpath ex)
    {
        ROS_INFO("arm_ctrl_navigate::Plannedpath exception received during the execution of GetRowDistance function inside path planner node");
        return 0;
    }
    catch (...)
    {
        ROS_INFO("Exception received during the execution of GetRowDistance function inside path planner node");
        return 0;
    }
}

void LogPointsToFile(arm_ctrl_navigate::Path::Ptr trajectory, std::string file_name)//detachable
{

    try
    {
        ofstream outdata;
 
        outdata.open(file_name);
        if (!outdata)
        { // file couldn't be opened
            cerr << "Error: file could not be opened" << endl;
            exit(1);
        }

        int total_points = 0;
        for (int i = 0; i < trajectory->path.size(); i++)
        {
            outdata << "Row number = " << i << endl;
            for (int j = 0; j < trajectory->path[i].path_msg.size(); ++j)
            {
                ////For logging with label
                //outdata << "x = " << trajectory->path[i].path_msg[j].x << " y = " << trajectory->path[i].path_msg[j].y << " z = " << trajectory->path[i].path_msg[j].z << " roll = " << trajectory->path[i].path_msg[j].u << " pitch = " << trajectory->path[i].path_msg[j].v << " yaw = " << trajectory->path[i].path_msg[j].w << endl;
                //For Arm format
                //outdata << ((trajectory->path[i].path_msg[j].x ) ) << "    " << ( trajectory->path[i].path_msg[j].y ) << "    " << ( trajectory->path[i].path_msg[j].z ) << "    " << trajectory->path[i].path_msg[j].u << trajectory->path[i].path_msg[j].v << trajectory->path[i].path_msg[j].w << endl;
                outdata << ((trajectory->path[i].path_msg[j].x ) ) << "    " << ( trajectory->path[i].path_msg[j].y ) << "    " << ( trajectory->path[i].path_msg[j].z ) << "    " <<(trajectory->path[i].path_msg[j].ox )<< "    " <<(trajectory->path[i].path_msg[j].oy )<< "    " <<(trajectory->path[i].path_msg[j].oz )<< "    " <<(trajectory->path[i].path_msg[j].ow ) <<std::endl;
                //For terminal print
                //ROS_INFO("x = %f y = %f z = %f u = %f v = %f w = %f",trajectory->path[i].path_msg[j].x,trajectory->path[i].path_msg[j].y,trajectory->path[i].path_msg[j].z,trajectory->path[i].path_msg[j].u,trajectory->path[i].path_msg[j].v,trajectory->path[i].path_msg[j].w);
                total_points++;
            }

            //ROS_INFO("Row number = %d points = %d",i,trajectory->path[i].path_msg.size());
            outdata << "Number of points in the row = " << trajectory->path[i].path_msg.size() << endl;
        }
        outdata << "Total Number of points = " << total_points << endl;
        outdata.close();
    }
    catch (...)
    {
        ROS_INFO("Exception received during the execution of LogPointsToFile function inside path planner node");
    }
}

void LogToFile(std::vector<double> val, std::string file_name)//detachable
{

    try
    {
        ofstream outdata;
 
        outdata.open(file_name);
        if (!outdata)
        { // file couldn't be opened
            cerr << "Error: file could not be opened" << endl;
            exit(1);
        }

        for(int i = 0; i < val.size() ; i++){
            outdata << val[i] <<std::endl;
        }
        outdata.close();
    }
    catch (...)
    {
        ROS_INFO("Exception received during the execution of LogPointsToFile function inside path planner node");
    }
}


MinMaxX FindminmaxX(arm_ctrl_navigate::Path::Ptr trajectory)
{
    float minx = 10.0;
    float maxx = 0.0;
    try
    {
        for (int i = 0; i < trajectory->path.size(); i++)
        {
            
            for (int j = 0; j < trajectory->path[i].path_msg.size(); j++)
            {
   
                if((trajectory->path[i].path_msg[j].x < minx) && (trajectory->path[i].path_msg[j].x>-0.6)){minx = trajectory->path[i].path_msg[j].x;}
                else if(trajectory->path[i].path_msg[j].x > maxx && trajectory->path[i].path_msg[j].x<1.5){maxx = trajectory->path[i].path_msg[j].x;}
            }
            
        }
    MinMaxX xval;
    xval.minx = minx;
    xval.maxx = maxx;
    ROS_INFO("minimum x and maximum x %.2f   %.2f",minx,maxx);
    return xval;
    }
    catch (...)
    {
        ROS_INFO("Exception received during the execution of FindminmaxX function inside path planner node");
    }
}