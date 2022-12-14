#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <eigen_conversions/eigen_msg.h>
#include <iostream>
#include <fstream>

#include <std_msgs/Bool.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
// Means by which we communicate with above action-server
#include <actionlib/client/simple_action_client.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <trajectory_planning/JointSpaceTrajectoryService.h>
#include <trajectory_planning/CustomTrajectoryServiceWithoutIndex.h>

#include <trajectory_planning/CoatTrajectoryFeasiblity.h>
#include <trajectory_planning/CoatCompletionStatus.h>

#include <trajectory_planning/StartPainting.h>
#include <trajectory_planning/StartPaintingRequest.h>
#include <trajectory_planning/StartPaintingResponse.h>

#include <trajectory_planning/StartTrajCalculation.h>
#include <trajectory_planning/StartTrajCalculationRequest.h>
#include <trajectory_planning/StartTrajCalculationResponse.h>

#include <pointcloud_process/ApplyTF.h>
#include <pointcloud_process/ApplyTFResponse.h>
#include <pointcloud_process/ApplyTFRequest.h>
#include <rtde_python/ReceiveDataUR.h>
#include <rtde_python/ReceiveDataURResponse.h>
#include <rtde_python/SendDataUR.h>
#include <rtde_python/ReceiveDataURResponse.h>

#include <arm_ctrl_navigate/Path.h>

bool sealer_read = false;
bool base1_read = false;
bool base2_read = false;
bool clear1_read = false;
bool clear2_read = false;

bool sealer_exec = false;
bool base1_exec = false;
bool base2_exec = false;
bool clear1_exec = false;
bool clear2_exec = false;
double speed_val = 368.00;
bool trajectory_calculation_completed = false;
bool restart_flag = false;
bool run = true;
bool dont_run = false;
std_msgs::Bool trajectory_calculation_completed_, painting_status;

std::vector<double> joint_vel_g;

trajectory_planning::JointSpaceTrajectoryServiceResponse trajectory_sealer;
trajectory_planning::JointSpaceTrajectoryServiceResponse trajectory_base1;
trajectory_planning::JointSpaceTrajectoryServiceResponse trajectory_base2;
trajectory_planning::JointSpaceTrajectoryServiceResponse trajectory_clear1;
trajectory_planning::JointSpaceTrajectoryServiceResponse trajectory_clear2;

arm_ctrl_navigate::Path sealerCoat;
arm_ctrl_navigate::Path base1Coat;
arm_ctrl_navigate::Path base2Coat;
arm_ctrl_navigate::Path clear1Coat;
arm_ctrl_navigate::Path clear2Coat;

arm_ctrl_navigate::Path sealer_wrt_mir;
arm_ctrl_navigate::Path base1_wrt_mir;
arm_ctrl_navigate::Path base2_wrt_mir;
arm_ctrl_navigate::Path clear1_wrt_mir;
arm_ctrl_navigate::Path clear2_wrt_mir;

trajectory_planning::CoatTrajectoryFeasiblity coat_feas_stats;
trajectory_planning::CoatCompletionStatus coat_completion_stats;

void doneCb(const actionlib::SimpleClientGoalState &state,
            const control_msgs::FollowJointTrajectoryResultConstPtr &result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Answer:");
}

// Called once when the goal becomes active
void activeCb()
{
  ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void feedbackCb(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &feedback)
{

  ROS_INFO("Got Feedback of length ");
  std::vector<double> joint_vel;
  for (int i = 0; i < 6; i++)
  {
    std::cout << feedback->actual.velocities[i];
    joint_vel.push_back(feedback->actual.velocities[i]);
  }

  joint_vel_g = joint_vel;
}

bool executeTrajectory(const trajectory_msgs::JointTrajectory &trajectory)
{
  // Create a Follow Joint Trajectory Action Client
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("/scaled_pos_joint_traj_controller/follow_joint_trajectory", true);
  if (!ac.waitForServer(ros::Duration(2.0)))
  {
    ROS_ERROR("Could not connect to action server");
    return false;
  }

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  goal.goal_time_tolerance = ros::Duration(1.0);

  ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

  if (ac.waitForResult(goal.trajectory.points[goal.trajectory.points.size() - 1].time_from_start + ros::Duration(5)))
  {
    ROS_INFO("Action server reported successful execution");
    return true;
  }
  else
  {
    ROS_WARN("Action server could not execute trajectory");
    return false;
  }
}

void sealer_callback(const arm_ctrl_navigate::Path::ConstPtr &path)
{
  // std::cout << "reached" << std::endl;
  if (!sealer_read)
  {
    sealerCoat.path.clear();
    std::cout<<"sealerCoat.path.size() : "<<sealerCoat.path.size()<<std::endl;
    
    sealerCoat.CoatSpeed = path->CoatSpeed;
    sealerCoat.CoatName = path->CoatName ;
    sealerCoat.IndexDelay = path->IndexDelay;
    sealerCoat.path = path->path;
    int count = 0;
    for (int i = 0;i<path->path.size();i++){
        for(int j = 0;j<path->path[i].path_msg.size();j++){
            count++;   
        }  
    }
    std::cout << "sealer_callback count = " << count << std::endl;

    sealer_read = true;
  }
}

void base1_callback(const arm_ctrl_navigate::Path::ConstPtr &path)
{
  if (!base1_read)
  {
    base1Coat.path.clear();
    base1Coat.CoatSpeed = path->CoatSpeed;
    base1Coat.CoatName = path->CoatName ;
    base1Coat.IndexDelay = path->IndexDelay;
    base1Coat.path = path->path;

    int count = 0;
    for (int i = 0;i<path->path.size();i++){
        for(int j = 0;j<path->path[i].path_msg.size();j++){
            count++;   
        }  
    }
    std::cout << "base1_callback count = " << count << std::endl;

    base1_read = true;
  }
}
void base2_callback(const arm_ctrl_navigate::Path::ConstPtr &path)
{
  if (!base2_read)
  {
    base2Coat.path.clear();
    base2Coat.CoatSpeed = path->CoatSpeed;
    base2Coat.CoatName = path->CoatName ;
    base2Coat.IndexDelay = path->IndexDelay;
    base2Coat.path = path->path;

    int count = 0;
    for (int i = 0;i<path->path.size();i++){
        for(int j = 0;j<path->path[i].path_msg.size();j++){
            count++;   
        }  
    }
    std::cout << "base2_callback count = " << count << std::endl;

    base2_read = true;
  }
}
void clear1_callback(const arm_ctrl_navigate::Path::ConstPtr &path)
{
  if (!clear1_read)
  {
    clear1Coat.path.clear();
    clear1Coat.CoatSpeed = path->CoatSpeed;
    clear1Coat.CoatName = path->CoatName ;
    clear1Coat.IndexDelay = path->IndexDelay;
    clear1Coat.path = path->path;

    int count = 0;
    for (int i = 0;i<path->path.size();i++){
        for(int j = 0;j<path->path[i].path_msg.size();j++){
            count++;   
        }  
    }
    std::cout << "clear1_callback count = " << count << std::endl;

    
    clear1_read = true;
  }
}
void clear2_callback(const arm_ctrl_navigate::Path::ConstPtr &path)
{
  if (!clear2_read)
  {
    clear2Coat.path.clear();
    clear2Coat.CoatSpeed = path->CoatSpeed;
    clear2Coat.CoatName = path->CoatName ;
    clear2Coat.IndexDelay = path->IndexDelay;
    clear2Coat.path = path->path;

    int count = 0;
    for (int i = 0;i<path->path.size();i++){
        for(int j = 0;j<path->path[i].path_msg.size();j++){
            count++;   
        }  
    }
    std::cout << "clear2_callback count = " << count << std::endl;


    clear2_read = true;
  }
}

pointcloud_process::ApplyTFResponse caller(ros::NodeHandle &n, ros::ServiceClient &apply_tf_client, pointcloud_process::ApplyTF &srv_atf, arm_ctrl_navigate::Path &inputPathPoints)
{

  int count = 0;
  for (int i = 0;i<inputPathPoints.path.size();i++){
      for(int j = 0;j<inputPathPoints.path[i].path_msg.size();j++){
          count++;   
      }  
  }
  std::cout << "ApplyTFResponse caller count = " << count << std::endl;

  srv_atf.request.in_path = inputPathPoints;
  srv_atf.request.coat_name = "sealer";
  srv_atf.request.speed = 490;
  srv_atf.request.delay = 1.0;
  apply_tf_client.call(srv_atf);

  return srv_atf.response;
}

trajectory_planning::JointSpaceTrajectoryServiceResponse caller(ros::NodeHandle &n, ros::ServiceClient &joint_space_trajectory_client, ros::ServiceClient &custom_traj_without_index_client, trajectory_planning::JointSpaceTrajectoryService &srv_jst, trajectory_planning::CustomTrajectoryServiceWithoutIndex &srv_ctswi, arm_ctrl_navigate::Path &inputPathPoints)
{
  int count = 0;
  for (int i = 0;i<inputPathPoints.path.size();i++){
      for(int j = 0;j<inputPathPoints.path[i].path_msg.size();j++){
          count++;   
      }  
  }
  std::cout << "JointSpaceTrajectoryServiceResponse caller count = " << count << std::endl;

  srv_ctswi.request.path = inputPathPoints;
  // std::cout << "srv_ctswi : " << srv_ctswi.request.path << std::endl;
  
  custom_traj_without_index_client.call(srv_ctswi);
  
  ROS_INFO("type change service returned");
  std::vector<geometry_msgs::Pose> waypoints;
  int cnt = 0;
  for (int i = 0; i < srv_ctswi.response.trajectory_response.poses.size(); i++)
  {
    waypoints.push_back(srv_ctswi.response.trajectory_response.poses[i]);
    // std::cout << srv_ctswi.response.trajectory_response.poses[i] << std::endl;
    cnt +=1;
  }
  std::cout<<"-----------------------------------------count : "<<cnt<<"---------------------------------------  "<<std::endl;
  srv_jst.request.waypoints = waypoints;
  ROS_INFO("calling trajectory service.............");
  joint_space_trajectory_client.call(srv_jst);
  return srv_jst.response;
}

bool handle_start_painting_service(trajectory_planning::StartPainting::Request &req,
                                   trajectory_planning::StartPainting::Response &res)
{

  if (req.start == "sealer")
  {
    ros::param::set("exec_sealer", true);
    res.sealer = true;
    res.base1 = false;
    res.base2 = false;
    res.clear1 = false;
    res.clear2 = false;
  }
  else if (req.start == "base1")
  {

    ros::param::set("exec_base1", true);
    res.sealer = false;
    res.base1 = true;
    res.base2 = false;
    res.clear1 = false;
    res.clear2 = false;
  }
  else if (req.start == "base2")
  {

    ros::param::set("exec_base2", true);
    res.sealer = false;
    res.base1 = false;
    res.base2 = true;
    res.clear1 = false;
    res.clear2 = false;
  }
  else if (req.start == "clear1")
  {

    ros::param::set("exec_clear1", true);
    res.sealer = false;
    res.base1 = false;
    res.base2 = false;
    res.clear1 = true;
    res.clear2 = false;
  }
  else if (req.start == "clear2")
  {

    ros::param::set("exec_clear2", true);
    res.sealer = false;
    res.base1 = false;
    res.base2 = false;
    res.clear1 = false;
    res.clear2 = true;
  }
  return true;
}

bool handle_start_trajectory_planning_calculation_service(trajectory_planning::StartTrajCalculation::Request &req,
                                                          trajectory_planning::StartTrajCalculation::Response &res)
{
  ros::param::set("start_trajectory_calculation", true);
  ros::param::set("axalta/ccscore/dashboard/CURRENT_PROCESS","Trajectory Planning is in progress...");
  ros::param::set("axalta/ccscore/dashboard/COMPLETION_PERCENTAGE", 50);
  res.resp = true;
  return true;
}

void restartNode(ros::NodeHandle &n)
{
  ROS_ERROR("restarting trajectory executor");
  sealer_read = false;
  base1_read = false;
  base2_read = false;
  clear1_read = false;
  clear2_read = false;

  sealer_exec = false;
  base1_exec = false;
  base2_exec = false;
  clear1_exec = false;
  clear2_exec = false;

  run = true;
  dont_run = false;
  trajectory_calculation_completed = false;
  restart_flag = false;

  trajectory_calculation_completed_.data = false;
  painting_status.data = false;

  joint_vel_g.clear();

  trajectory_sealer.coat_feasibility = false;
  trajectory_base1.coat_feasibility = false;
  trajectory_base2.coat_feasibility = false;
  trajectory_clear1.coat_feasibility = false;
  trajectory_clear2.coat_feasibility = false;

  trajectory_sealer.coat_ik_percentage = 0.0;
  trajectory_base1.coat_ik_percentage = 0.0;
  trajectory_base2.coat_ik_percentage = 0.0;
  trajectory_clear1.coat_ik_percentage = 0.0;
  trajectory_clear2.coat_ik_percentage = 0.0;

  trajectory_sealer.trajectory.joint_trajectory.points.clear();
  trajectory_sealer.trajectory.joint_trajectory.joint_names.clear();
  trajectory_sealer.poses_fk.poses.clear();

  trajectory_base1.trajectory.joint_trajectory.points.clear();
  trajectory_base1.trajectory.joint_trajectory.joint_names.clear();
  trajectory_base1.poses_fk.poses.clear();

  trajectory_base2.trajectory.joint_trajectory.points.clear();
  trajectory_base2.trajectory.joint_trajectory.joint_names.clear();
  trajectory_base2.poses_fk.poses.clear();

  trajectory_clear1.trajectory.joint_trajectory.points.clear();
  trajectory_clear1.trajectory.joint_trajectory.joint_names.clear();
  trajectory_clear1.poses_fk.poses.clear();

  trajectory_clear2.trajectory.joint_trajectory.points.clear();
  trajectory_clear2.trajectory.joint_trajectory.joint_names.clear();
  trajectory_clear2.poses_fk.poses.clear();

  coat_completion_stats.sealer = false;
  coat_completion_stats.base1 = false;
  coat_completion_stats.base2 = false;
  coat_completion_stats.clear1 = false;
  coat_completion_stats.clear2 = false;

  sealerCoat.path.clear();
  sealerCoat.CoatSpeed = 0.0; 
  sealerCoat.CoatName = "";
  sealerCoat.IndexDelay = 0;
  base1Coat.path.clear();
  base1Coat.CoatSpeed = 0.0; 
  base1Coat.CoatName = "";
  base1Coat.IndexDelay = 0; 
  base2Coat.path.clear(); 
  base2Coat.CoatSpeed = 0.0; 
  base2Coat.CoatName = "";
  base2Coat.IndexDelay = 0;
  clear1Coat.path.clear();
  clear1Coat.CoatSpeed = 0.0; 
  clear1Coat.CoatName = "";
  clear1Coat.IndexDelay = 0;
  clear2Coat.path.clear();
  clear2Coat.CoatSpeed = 0.0; 
  clear2Coat.CoatName = "";
  clear2Coat.IndexDelay = 0;

  sealer_wrt_mir.path.clear();
  sealer_wrt_mir.CoatSpeed = 0.0; 
  sealer_wrt_mir.CoatName = "";
  sealer_wrt_mir.IndexDelay = 0;
  base1_wrt_mir.path.clear();
  base1_wrt_mir.CoatSpeed = 0.0; 
  base1_wrt_mir.CoatName = "";
  base1_wrt_mir.IndexDelay = 0; 
  base2_wrt_mir.path.clear(); 
  base2_wrt_mir.CoatSpeed = 0.0; 
  base2_wrt_mir.CoatName = "";
  base2_wrt_mir.IndexDelay = 0;
  clear1_wrt_mir.path.clear();
  clear1_wrt_mir.CoatSpeed = 0.0; 
  clear1_wrt_mir.CoatName = "";
  clear1_wrt_mir.IndexDelay = 0;
  clear2_wrt_mir.path.clear();
  clear2_wrt_mir.CoatSpeed = 0.0; 
  clear2_wrt_mir.CoatName = "";
  clear2_wrt_mir.IndexDelay = 0;


  if (n.hasParam("axalta/ccscore/dashboard/restart_trajectory_executor_node_trigger"))
  {
    n.setParam("axalta/ccscore/dashboard/restart_trajectory_executor_node_trigger", false);
    restart_flag = false;
    ros::Duration(0.1).sleep();
    n.setParam("axalta/ccscore/dashboard/restart_final_receData_ur_node_trigger", true);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_executor");

  ros::NodeHandle n;
  ros::param::set("Processing_coat", "None");
  ros::param::set("exec_sealer", false);
  ros::param::set("exec_base1", false);
  ros::param::set("exec_base2", false);
  ros::param::set("exec_clear1", false);
  ros::param::set("exec_clear2", false);

  ros::param::set("start_trajectory_calculation", false);

  trajectory_calculation_completed_.data = false;
  painting_status.data = false;
  coat_completion_stats.sealer = false;
  coat_completion_stats.base1 = false;
  coat_completion_stats.base2 = false;
  coat_completion_stats.clear1 = false;
  coat_completion_stats.clear2 = false;

  n.setParam("axalta/ccscore/dashboard/restart_trajectory_executor_node_trigger", false);
  n.setParam("axalta/ccscore/dashboard/restart_pointcloud_and_arm_completed", false);
  ros::Rate loop_rate(10);
  ros::AsyncSpinner spinner(0);
  spinner.start();

  ros::Subscriber sealer_sub = n.subscribe("/shifted_point_sealercoat", 5, sealer_callback);
  ros::Subscriber base1_sub = n.subscribe("/shifted_point_basecoat1", 5, base1_callback);
  ros::Subscriber base2_sub = n.subscribe("/shifted_point_basecoat2", 5, base2_callback);
  ros::Subscriber clear1_sub = n.subscribe("/shifted_point_clearcoat1", 5, clear1_callback);
  ros::Subscriber clear2_sub = n.subscribe("/shifted_point_clearcoat2", 5, clear2_callback);

  ros::Publisher feasibility_pub = n.advertise<trajectory_planning::CoatTrajectoryFeasiblity>("coats_feasibility", 10);
  ros::Publisher coat_completion_status_pub = n.advertise<trajectory_planning::CoatCompletionStatus>("coat_completion_status", 10);
  ros::Publisher trajectory_planning_status_pub = n.advertise<std_msgs::Bool>("trajectory_planning_status", 10);
  ros::Publisher painting_status_pub = n.advertise<std_msgs::Bool>("painting_status", 10);

  ROS_INFO("subscribers initialized");
  ros::ServiceServer painting_service = n.advertiseService("start_painting_service", handle_start_painting_service);
  ros::ServiceServer traj_calc_service = n.advertiseService("start_trajectory_planning_calculation", handle_start_trajectory_planning_calculation_service);
  ros::ServiceClient joint_space_trajectory_client = n.serviceClient<trajectory_planning::JointSpaceTrajectoryService>("trajectory_planning_service");
  ros::ServiceClient custom_traj_without_index_client = n.serviceClient<trajectory_planning::CustomTrajectoryServiceWithoutIndex>("trajectory_type_conversion_without_index");
  ros::ServiceClient apply_tf_client = n.serviceClient<pointcloud_process::ApplyTF>("apply_tf_server");
  ros::ServiceClient receive_data_ur_client = n.serviceClient<rtde_python::ReceiveDataUR>("receive_data_ur_server");
  ros::ServiceClient send_data_ur_client = n.serviceClient<rtde_python::SendDataUR>("send_data_ur_server");
  //ros::ServiceClient ccs_mir_door_client = n.serviceClient<ccs_lite_communication::mirDoorAction>("ccs_lite_mir_door_action_server");
  ROS_INFO("created service clients");
  trajectory_planning::JointSpaceTrajectoryService srv_jst;
  trajectory_planning::CustomTrajectoryServiceWithoutIndex srv_ctswi;
  rtde_python::SendDataUR srv_sdur;
  rtde_python::ReceiveDataUR srv_rdur;
  pointcloud_process::ApplyTF srv_atf;
  //ccs_lite_communicate::mirDoorAction
  
  while (ros::ok())
  {

    if (n.hasParam("axalta/ccscore/dashboard/restart_trajectory_executor_node_trigger") && n.getParam("axalta/ccscore/dashboard/restart_trajectory_executor_node_trigger", restart_flag))
    {
      if (restart_flag)
      {
        ROS_INFO("Received restart request. Restarting trajectory_executor Node...");
        restartNode(n);
      }
    }

    ros::param::get("exec_sealer", sealer_exec);
    ros::param::get("exec_base1", base1_exec);
    ros::param::get("exec_base2", base2_exec);
    ros::param::get("exec_clear1", clear1_exec);
    ros::param::get("exec_clear2", clear2_exec);

    if (sealer_read && base1_read && base2_read && clear1_read && clear2_read && run)
    { //&& base1_read && base2_read && clear1_read && clear2_read && run){
      ROS_INFO("Received all Coat path points");

      ROS_INFO("Sealer Processing ... ");
      ros::param::set("Processing_coat", "sealer");
      ros::param::set("record_vid_name", "sealer");
      trajectory_sealer = caller(n, joint_space_trajectory_client, custom_traj_without_index_client, srv_jst, srv_ctswi, sealerCoat);
      ROS_INFO("----------------------Sealer Done--------------------------------------------------------------------------------------------------------------");

      ROS_INFO("BaseCoat1 Processing ... ");
      ros::param::set("Processing_coat", "base1");
      ros::param::set("record_vid_name", "base1");
      trajectory_base1 = caller(n, joint_space_trajectory_client, custom_traj_without_index_client, srv_jst, srv_ctswi, base1Coat);
      ROS_INFO("----------------------BaseCoat1 Done--------------------------------------------------------------------------------------------------------------");

      ROS_INFO("BaseCoat2 Processing ... ");
      ros::param::set("Processing_coat", "base2");
      ros::param::set("record_vid_name", "base2");
      trajectory_base2 = caller(n, joint_space_trajectory_client, custom_traj_without_index_client, srv_jst, srv_ctswi, base2Coat);
      ROS_INFO("----------------------BaseCoat2 Done--------------------------------------------------------------------------------------------------------------");
      ROS_INFO("ClearCoat1 Processing ... ");
      ros::param::set("record_vid_name", "clear1");
      ros::param::set("Processing_coat", "clear1");
      trajectory_clear1 = caller(n, joint_space_trajectory_client, custom_traj_without_index_client, srv_jst, srv_ctswi, clear1Coat);
      ROS_INFO("----------------------ClearCoat1 Done--------------------------------------------------------------------------------------------------------------");

      ROS_INFO("ClearCoat2 Processing ... ");
      ros::param::set("Processing_coat", "clear2");
      ros::param::set("record_vid_name", "clear2");
      trajectory_clear2 = caller(n, joint_space_trajectory_client, custom_traj_without_index_client, srv_jst, srv_ctswi, clear2Coat);
      ROS_INFO("----------------------ClearCoat2 Done--------------------------------------------------------------------------------------------------------------");

      trajectory_calculation_completed = true;
      trajectory_calculation_completed_.data = true;
      ros::param::set("axalta/ccscore/dashboard/CURRENT_PROCESS","Trajectory planning has completed");
      ros::param::set("axalta/ccscore/dashboard/COMPLETION_PERCENTAGE", 100);
      run = false;
    }
    

    if (trajectory_calculation_completed)
    {
      if (!trajectory_sealer.coat_feasibility || !trajectory_base1.coat_feasibility || !trajectory_base2.coat_feasibility || !trajectory_clear1.coat_feasibility)
      {
        dont_run = true;
      }
      if (!dont_run)
      {
        // ccs_mir_door_client.call();
        if (sealer_exec)
        {
          sealer_wrt_mir = caller(n, apply_tf_client, srv_atf, sealerCoat).out_path;
          srv_rdur.request.trajectory_fk = trajectory_sealer.poses_fk;
          srv_rdur.request.current_coat_path = sealerCoat;
          srv_rdur.request.current_coat_path_transformed = sealer_wrt_mir;
          receive_data_ur_client.call(srv_rdur);
          srv_sdur.request.list_each_point = srv_rdur.response.list_each_point;
          srv_sdur.request.trajectory = trajectory_sealer.trajectory;
          if (n.hasParam("axalta/ccscore/dashboard/SPRAYGUN_TRAVERSE_SPEED_sealercoat"))
            {n.getParam("axalta/ccscore/dashboard/SPRAYGUN_TRAVERSE_SPEED_sealercoat", speed_val);}    
          srv_sdur.request.speed = speed_val;
          send_data_ur_client.call(srv_sdur);
          coat_completion_stats.sealer = true;

          ros::param::set("exec_sealer", false);
        }
        else if (base1_exec)
        {
          base1_wrt_mir = caller(n, apply_tf_client, srv_atf, base1Coat).out_path;
          srv_rdur.request.trajectory_fk = trajectory_base1.poses_fk;
          srv_rdur.request.current_coat_path = base1Coat;
          srv_rdur.request.current_coat_path_transformed = base1_wrt_mir;
          receive_data_ur_client.call(srv_rdur);
          srv_sdur.request.list_each_point = srv_rdur.response.list_each_point;
          srv_sdur.request.trajectory = trajectory_base1.trajectory;
          if (n.hasParam("axalta/ccscore/dashboard/SPRAYGUN_TRAVERSE_SPEED_basecoat1"))
            {n.getParam("axalta/ccscore/dashboard/SPRAYGUN_TRAVERSE_SPEED_basecoat1", speed_val);}  
          srv_sdur.request.speed = speed_val;
          send_data_ur_client.call(srv_sdur);
          coat_completion_stats.base1 = true;

          ros::param::set("exec_base1", false);
        }
        else if (base2_exec)
        {
          base2_wrt_mir = caller(n, apply_tf_client, srv_atf, base2Coat).out_path;
          srv_rdur.request.trajectory_fk = trajectory_base2.poses_fk;
          srv_rdur.request.current_coat_path = base2Coat;
          srv_rdur.request.current_coat_path_transformed = base2_wrt_mir;
          receive_data_ur_client.call(srv_rdur);
          srv_sdur.request.list_each_point = srv_rdur.response.list_each_point;
          srv_sdur.request.trajectory = trajectory_base2.trajectory;
          if (n.hasParam("axalta/ccscore/dashboard/SPRAYGUN_TRAVERSE_SPEED_basecoat2"))
            {n.getParam("axalta/ccscore/dashboard/SPRAYGUN_TRAVERSE_SPEED_basecoat2", speed_val);}     
          srv_sdur.request.speed = speed_val;
          send_data_ur_client.call(srv_sdur);
          coat_completion_stats.base2 = true;

          ros::param::set("exec_base2", false);
        }
        else if (clear1_exec)
        {
          clear1_wrt_mir = caller(n, apply_tf_client, srv_atf, clear1Coat).out_path;
          srv_rdur.request.trajectory_fk = trajectory_clear1.poses_fk;
          srv_rdur.request.current_coat_path = clear1Coat;
          srv_rdur.request.current_coat_path_transformed = clear1_wrt_mir;
          receive_data_ur_client.call(srv_rdur);
          srv_sdur.request.list_each_point = srv_rdur.response.list_each_point;
          srv_sdur.request.trajectory = trajectory_clear1.trajectory;

          if (n.hasParam("axalta/ccscore/dashboard/SPRAYGUN_TRAVERSE_SPEED_clearcoat1"))
            {n.getParam("axalta/ccscore/dashboard/SPRAYGUN_TRAVERSE_SPEED_clearcoat1", speed_val);}    
          srv_sdur.request.speed = speed_val;
                
          send_data_ur_client.call(srv_sdur);
          coat_completion_stats.clear1 = true;

          ros::param::set("exec_clear1", false);
        }
        else
        {
          //std::cout << "waiting for execution trigger condtion " << std::endl;
        }
      }
      else
      {
        //ROS_WARN("One or more Coats not feasible");
      }
      if (trajectory_clear2.coat_feasibility && clear2_exec)
      {
        clear2_wrt_mir = caller(n, apply_tf_client, srv_atf, clear2Coat).out_path;
        srv_rdur.request.trajectory_fk = trajectory_clear2.poses_fk;
        srv_rdur.request.current_coat_path = clear2Coat;
        srv_rdur.request.current_coat_path_transformed = clear2_wrt_mir;
        receive_data_ur_client.call(srv_rdur);
        srv_sdur.request.list_each_point = srv_rdur.response.list_each_point;
        srv_sdur.request.trajectory = trajectory_clear2.trajectory;
        if (n.hasParam("axalta/ccscore/dashboard/SPRAYGUN_TRAVERSE_SPEED_clearcoat2"))
          {n.getParam("axalta/ccscore/dashboard/SPRAYGUN_TRAVERSE_SPEED_clearcoat2", speed_val);}       
        srv_sdur.request.speed = speed_val;

        send_data_ur_client.call(srv_sdur);
                
        coat_completion_stats.clear2 = true;
        ros::param::set("exec_clear2", false);
      }
      if (!trajectory_clear2.coat_feasibility)
      {
        //ROS_WARN("clear2 coat is not feasible");
      }
    }
    if (coat_completion_stats.sealer && coat_completion_stats.base1 && coat_completion_stats.base2 && coat_completion_stats.clear1 && coat_completion_stats.clear2)
    {
      painting_status.data = true;
      ros::param::set("axalta/ccscore/dashboard/CURRENT_PROCESS","Painting process has completed");
      ros::param::set("axalta/ccscore/dashboard/COMPLETION_PERCENTAGE", 100);
    }
    coat_feas_stats.sealer_coat_feasiblity = trajectory_sealer.coat_feasibility;
    coat_feas_stats.base1_coat_feasiblity = trajectory_base1.coat_feasibility;
    coat_feas_stats.base2_coat_feasiblity = trajectory_base2.coat_feasibility;
    coat_feas_stats.clear1_coat_feasiblity = trajectory_clear1.coat_feasibility;
    coat_feas_stats.clear2_coat_feasiblity = trajectory_clear2.coat_feasibility;

    coat_feas_stats.sealer_coat_ik_percentage = std::to_string((int)trajectory_sealer.coat_ik_percentage)+"%";
    coat_feas_stats.base1_coat_ik_percentage = std::to_string((int)trajectory_base1.coat_ik_percentage)+"%";
    coat_feas_stats.base2_coat_ik_percentage = std::to_string((int)trajectory_base2.coat_ik_percentage)+"%";
    coat_feas_stats.clear1_coat_ik_percentage = std::to_string((int)trajectory_clear1.coat_ik_percentage)+"%";
    coat_feas_stats.clear2_coat_ik_percentage = std::to_string((int)trajectory_clear2.coat_ik_percentage)+"%";

    feasibility_pub.publish(coat_feas_stats);
    coat_completion_status_pub.publish(coat_completion_stats);
    trajectory_planning_status_pub.publish(trajectory_calculation_completed_);
    painting_status_pub.publish(painting_status);

    loop_rate.sleep();
  }

  return 0;
}
