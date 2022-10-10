#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <eigen_conversions/eigen_msg.h>
#include <iostream>
#include <fstream>

#include <chrono>
#include <thread>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
//#include <control_msgs/FollowJointTrajectoryActionFeedback.h>
//#include <control_msgs/FollowJointTrajectoryActionResult.h>
// Means by which we communicate with above action-server
#include <actionlib/client/simple_action_client.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <trajectory_planning/JointSpaceTrajectoryService.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include "arm_ctrl_navigate/JointPath.h"
#include "arm_ctrl_navigate/JointValues.h"

// my_custom_msg_package::MyCustomMsg msg;
bool received = false;
bool calculation_done = false;
moveit_msgs::RobotTrajectory totg_trajectory_time_scaled_output;

moveit_msgs::RobotTrajectory totg_trajectory_continuous;
std::vector<geometry_msgs::Pose> waypts;
bool entered = false;
bool update_Processing_coat_done = false;
std::vector<std::string> joint_names;
bool coat_feasibility = false;
geometry_msgs::PoseArray poses;
// arm_ctrl_navigate::JointPath::ConstPtr joint_path(new arm_ctrl_navigate::JointPath);
bool restart_flag = false;

Eigen::VectorXd getCurrentCartesianVelocity(robot_state::RobotStatePtr kinematic_model, std::vector<double> joint_vel)
{

  const robot_state::JointModelGroup *joint_model_group_ = kinematic_model->getJointModelGroup("manipulator");
  Eigen::MatrixXd jacobian_matrix = kinematic_model->getJacobian(joint_model_group_, Eigen::Vector3d(0.0, 0.0, 0.0));
  Eigen::VectorXd jvel;
  jvel << joint_vel[0], joint_vel[1], joint_vel[2], joint_vel[3], joint_vel[4], joint_vel[5];
  Eigen::VectorXd cartesian_vel = jacobian_matrix * jvel;
  return cartesian_vel;
}

bool handle_trajectory_planning_service(trajectory_planning::JointSpaceTrajectoryService::Request &req,
                                        trajectory_planning::JointSpaceTrajectoryService::Response &res)
{
  coat_feasibility = false;
  received = true;
  ROS_INFO("Trajectory service call received");
  waypts = req.waypoints;
  while (!calculation_done)
  {
  }
  ROS_INFO("Trajectory service computation done");
  res.trajectory = totg_trajectory_time_scaled_output;
  // res.trajectory = totg_trajectory_continuous;
  res.poses_fk = poses;
  res.coat_feasibility = coat_feasibility;
  calculation_done = false;
  ROS_INFO("Trajectory service returning...............");
  return true;
}

bool trajExec(moveit::planning_interface::MoveGroupInterface &move_group_interface, std::vector<geometry_msgs::Pose> waypoints, moveit_msgs::RobotTrajectory &totg_trajectory, double jump_threshold = 3.0, double eef_step = 0.07)
{ // 2.25 0.02

  moveit_msgs::RobotTrajectory trajectory;
  static const std::string PLANNING_GROUP = "manipulator";
  moveit_msgs::MoveItErrorCodes *error_code;

  move_group_interface.setMaxAccelerationScalingFactor(1.2);
  std::vector<geometry_msgs::Pose> waypoints1;
  std::cout << "waypoints.size() : " << waypoints.size() << std::endl;
  for (int i = 0; i < 20; i++)
  {
    waypoints1.push_back(waypoints[i]);
  }
  std::cout << "trajectory size : " << trajectory.joint_trajectory.points.size() << std::endl;
  double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true, error_code);

  std::cout << "completed first try : " << error_code->val << std::endl;

  std::cout << "fraction : " << fraction << std::endl;

  int tries = 0;
  while ((fraction * 100.0) < 100.0 && tries < 3)
  {
    double fraction_in = 0.0;
    move_group_interface.stop();

    trajectory.joint_trajectory.points.clear();
    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

    fraction_in = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true, error_code);
    ros::param::set("IK_PERCENTAGE_COMPLETION", (fraction * 100.0));
    fraction = fraction_in;
    tries++;
  }

  if (fraction == 1.0)
  {
    coat_feasibility = true;
  }
  else
  {
    coat_feasibility = false;
  }
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

  std::cout << "--------------------------------------------Before time parameterization---------------------------------------------------------------" << std::endl;

  robot_trajectory::RobotTrajectory trajectory_for_iptp(move_group_interface.getRobotModel(), PLANNING_GROUP);
  robot_state::RobotState start_state(*move_group_interface.getCurrentState());
  trajectory_for_iptp.setRobotTrajectoryMsg(start_state, trajectory);
  ROS_INFO("IPTP started");
  double max_velocity_scaling_factor = 1.0;
  double max_acceleration_scaling_factor = 1.0;

  trajectory_processing::IterativeParabolicTimeParameterization time_speed_param(100, 0.01);
  time_speed_param.computeTimeStamps(trajectory_for_iptp, max_velocity_scaling_factor, max_acceleration_scaling_factor);

  moveit_msgs::RobotTrajectory iptp_trajectory;
  trajectory_for_iptp.getRobotTrajectoryMsg(iptp_trajectory);
  ROS_INFO("IPTP completed");

  robot_trajectory::RobotTrajectory trajectory_for_totg(move_group_interface.getRobotModel(), PLANNING_GROUP);

  robot_state::RobotState start_state1(*move_group_interface.getCurrentState());

  trajectory_for_totg.setRobotTrajectoryMsg(start_state1, iptp_trajectory);

  trajectory_for_totg.getRobotTrajectoryMsg(totg_trajectory);

  std::cout << "--------------------------------------------After time parameterization---------------------------------------------------------------" << std::endl;

  return true;
}

double distance_function(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2)
{
  return pow(pow((pose2.position.x - pose1.position.x), 2) + pow((pose2.position.y - pose1.position.y), 2) + pow((pose2.position.z - pose1.position.z), 2), 0.5);
}

void applyTimeOptimalTrajectoryGeneration(moveit::planning_interface::MoveGroupInterface &move_group_interface, const std::string PLANNING_GROUP, double required_sampling_time)
{

  robot_trajectory::RobotTrajectory trajectory_totg(move_group_interface.getRobotModel(), PLANNING_GROUP);

  robot_state::RobotState starting_state(*move_group_interface.getCurrentState());

  trajectory_totg.setRobotTrajectoryMsg(starting_state, totg_trajectory_time_scaled_output);

  // Get these from param server later
  double path_tolerance = 0.002;
  double resample_dt = required_sampling_time; // 0.03;
  double min_angle_change = 0.001;

  std::cout << "-----------------------required_sampling_time : " << required_sampling_time << std::endl;
  trajectory_processing::TimeOptimalTrajectoryGeneration time_param(path_tolerance, resample_dt, min_angle_change);
  time_param.computeTimeStamps(trajectory_totg, 1.0); // 1.0 accel scaling

  trajectory_totg.getRobotTrajectoryMsg(totg_trajectory_continuous);
}

trajectory_msgs::JointTrajectoryPoint calc_vel(trajectory_msgs::JointTrajectoryPoint pt1, trajectory_msgs::JointTrajectoryPoint pt2, double t1, double t2)
{

  trajectory_msgs::JointTrajectoryPoint result;
  result.positions = pt2.positions;
  ros::Duration dur(t2);
  result.time_from_start = dur;
  for (int i = 0; i < pt1.velocities.size(); i++)
  {
    result.velocities.push_back((pt2.positions[i] - pt1.positions[i]) / (t2 - t1));
  }
  return result;
}

trajectory_msgs::JointTrajectoryPoint calc_accel(trajectory_msgs::JointTrajectoryPoint pt1, trajectory_msgs::JointTrajectoryPoint pt2, double t1, double t2)
{

  trajectory_msgs::JointTrajectoryPoint result;
  result.positions = pt2.positions;
  result.velocities = pt2.velocities;
  ros::Duration dur(t2);
  result.time_from_start = dur;
  for (int i = 0; i < pt1.velocities.size(); i++)
  {
    result.accelerations.push_back((pt2.velocities[i] - pt1.velocities[i]) / (t2 - t1));
  }
  return result;
}

moveit_msgs::RobotTrajectory applyTimeScaling(moveit_msgs::RobotTrajectory &totg_trajectory, double ratio_bt_req_and_curr_time, int n)
{

  moveit_msgs::RobotTrajectory totg_trajectory_time_scaled;
  totg_trajectory_time_scaled.joint_trajectory = totg_trajectory.joint_trajectory;

  std::cout << n << std::endl;
  for (int i = 0; i < n; i++)
  {
    double timei = totg_trajectory.joint_trajectory.points[i].time_from_start.toSec();

    ros::Duration dur(timei * ratio_bt_req_and_curr_time);

    totg_trajectory_time_scaled.joint_trajectory.points[i].time_from_start = dur;
  }
  for (int i = 0; i < n - 1; i++)
  {
    totg_trajectory_time_scaled.joint_trajectory.points[i + 1] = calc_vel(totg_trajectory.joint_trajectory.points[i], totg_trajectory.joint_trajectory.points[i + 1], totg_trajectory_time_scaled.joint_trajectory.points[i].time_from_start.toSec(), totg_trajectory_time_scaled.joint_trajectory.points[i + 1].time_from_start.toSec());
  }

  for (int i = 0; i < n - 1; i++)
  {

    totg_trajectory_time_scaled.joint_trajectory.points[i + 1] = calc_accel(totg_trajectory_time_scaled.joint_trajectory.points[i], totg_trajectory_time_scaled.joint_trajectory.points[i + 1], totg_trajectory_time_scaled.joint_trajectory.points[i].time_from_start.toSec(), totg_trajectory_time_scaled.joint_trajectory.points[i + 1].time_from_start.toSec());
  }

  return totg_trajectory_time_scaled;
}

std::vector<Eigen::Affine3d> getFK(robot_state::RobotStatePtr kinematic_state, const robot_state::JointModelGroup *joint_model_group_n, moveit_msgs::RobotTrajectory totg_trajectory)
{

  std::vector<Eigen::Affine3d> waypts;
  Eigen::Affine3d waypoints_end_effector_state = Eigen::Affine3d(kinematic_state->getGlobalLinkTransform("wrist_3_link"));

  for (int i = 0; i < totg_trajectory.joint_trajectory.points.size(); i++)
  {
    kinematic_state->setJointGroupPositions(joint_model_group_n, totg_trajectory.joint_trajectory.points[i].positions);

    kinematic_state->enforceBounds();
    waypoints_end_effector_state = Eigen::Affine3d(kinematic_state->getGlobalLinkTransform("wrist_3_link"));
    waypts.push_back(waypoints_end_effector_state);
  }
  return waypts;
}

std::vector<geometry_msgs::Pose> trajectoryPoseEigenToMsg(std::vector<Eigen::Affine3d> waypts)
{

  std::vector<geometry_msgs::Pose> traj_vec;
  int i = 0;

  for (auto &pts : waypts)
  {
    // std::cout<<"waypoint : "<<i<<std::endl;
    geometry_msgs::Pose pose;

    tf::poseEigenToMsg(pts, pose);
    traj_vec.push_back(pose);

    i++;
  }
  return traj_vec;
}

geometry_msgs::PoseArray trajectoryPoseEigenToMsg_pose(std::vector<Eigen::Affine3d> waypts)
{

  geometry_msgs::PoseArray poses;
  int i = 0;

  for (auto &pts : waypts)
  {
    geometry_msgs::Pose pose;
    tf::poseEigenToMsg(pts, pose);
    poses.poses.push_back(pose);

    i++;
  }
  return poses;
}

void restartNode(ros::NodeHandle &node_handle)
{

  received = false;
  calculation_done = false;
  totg_trajectory_time_scaled_output.joint_trajectory.points.clear();

  totg_trajectory_continuous.joint_trajectory.points.clear();
  waypts.clear();
  entered = false;
  update_Processing_coat_done = false;
  joint_names.clear();
  coat_feasibility = false;

  if (node_handle.hasParam("axalta/ccscore/dashboard/restart_trajectory_planning_service_node_trigger"))
  {
    node_handle.setParam("axalta/ccscore/dashboard/restart_trajectory_planning_service_node_trigger", false);
    restart_flag = false;
    ros::Duration(0.1).sleep();
    node_handle.setParam("axalta/ccscore/dashboard/restart_trajectory_executor_node_trigger", true);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_planning_service");
  ros::NodeHandle node_handle;
  ros::param::set("IK_STATUS", "INCOMPLETE");
  ros::param::set("Processing_coat_done", "None");
  ros::AsyncSpinner spinner(0);
  spinner.start();

  node_handle.setParam("axalta/ccscore/dashboard/restart_trajectory_planning_service_node_trigger", false);

  ros::ServiceServer service = node_handle.advertiseService("trajectory_planning_service", handle_trajectory_planning_service);
  ros::Publisher final_trajectory_pub = node_handle.advertise<moveit_msgs::RobotTrajectory>("final_trajectory_", 10);
  ros::Publisher final_trajectoryTS_pub = node_handle.advertise<moveit_msgs::RobotTrajectory>("final_trajectory_timeScaled", 10);
  ros::Publisher final_trajectoryTO_pub = node_handle.advertise<moveit_msgs::RobotTrajectory>("final_trajectory_timeOptimized", 10);

  static const std::string PLANNING_GROUP = "manipulator";

  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  move_group_interface.setPlannerId("RRTConnect");
  move_group_interface.setPlanningTime(6);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  const robot_state::JointModelGroup *joint_model_group_n = kinematic_model->getJointModelGroup("manipulator");

  const moveit::core::JointModelGroup *joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  ros::Rate rate(10);

  while (ros::ok())
  {

    // std::cout<<"recv"<<std::endl;

    if (node_handle.hasParam("axalta/ccscore/dashboard/restart_calculate_end_effector_orientation_node_trigger") && node_handle.getParam("axalta/ccscore/dashboard/restart_calculate_end_effector_orientation_node_trigger", restart_flag))
    {
      if (restart_flag)
      {
        ROS_INFO("Received restart request. Restarting calculate_end_effector_orientation Node...");
        restartNode(node_handle);
      }
    }
    if(received){

    

    // std::cout<<"received : "<<std::endl;
    std::vector<double> jointval;

    // double arr[] = {-273.54*3.14/180.0, -100.64*3.14/180.0, 86.08*3.14/180.0,12.07*3.14/180.0, 88.93*3.14/180.0,180.0*3.14/180.0};//-4.091803375874655, -1.1082968574813385, 0.6583183447467249, 0.3286924797245483, 2.0630571842193604, -0.012504402791158498
    // double arr[] = {-4.091803375874655, -1.1082968574813385, 0.6583183447467249, 0.3286924797245483, 2.0630571842193604, -0.012504402791158498};//-0.955113712941305, -1.584445138970846, -1.8958847522735596, -2.830752512017721, -0.8862698713885706, -3.189246479664938
    // double arr[] = {-0.955113712941305, -1.584445138970846, -1.8958847522735596, -2.830752512017721, -0.8862698713885706, -3.189246479664938 };//
    // double arr[] = {1.5721356868743896, -1.7770129642882289, 1.6181653181659144, -2.861213823358053, -1.4771245161639612, 0.0};
    double arr[] = {1.0880, -1.735206, 1.76976, -2.94471, -1.06744, -0.097738};
    for (int iter = 0; iter < 6; iter++)
    {
      jointval.push_back(arr[iter]);
      std::cout << "arrval : " << arr[iter] << std::endl;
    }
    move_group_interface.setJointValueTarget(jointval);
    move_group_interface.move();

    ROS_INFO("In initial position--------");
    geometry_msgs::Pose start_pose2 = move_group_interface.getCurrentPose().pose;
    joint_names = move_group_interface.getJointNames();
    // for(auto &pts : jnames){
    //     std::cout<<pts<<std::endl;}

    std::cout << start_pose2 << std::endl;
    std::vector<geometry_msgs::Pose> waypointsall;
    std::vector<geometry_msgs::Pose> wpoints;
    geometry_msgs::Pose point_1;
    geometry_msgs::Pose point_2;
    geometry_msgs::Pose point_3;

    // wpoints.push_back(point_1);
    // wpoints.push_back(point_2);
    // wpoints.push_back(point_3);
    // wpoints.push_back(point_1);
    waypointsall = waypts;
    // waypointsall = wpoints;
    // waypointsall.insert(waypointsall.begin(), start_pose2);
    // waypointsall.push_back(start_pose2);
    moveit_msgs::RobotTrajectory totg_trajectory;
    ROS_INFO("Calling Trajectory Compute cartesian Path");
    trajExec(move_group_interface, waypointsall, totg_trajectory);

    ROS_INFO("--------cart_traj_done----------");

    std::vector<Eigen::Affine3d> waypts = getFK(kinematic_state, joint_model_group_n, totg_trajectory);

    std::vector<geometry_msgs::Pose> traj_vec = trajectoryPoseEigenToMsg(waypts);

    geometry_msgs::PoseArray poses_updated = trajectoryPoseEigenToMsg_pose(waypts);
    poses = poses_updated;

    double distance = 0.0;
    for (int i = 0; i < traj_vec.size() - 2; i++)
    {
      distance = distance + distance_function(traj_vec[i], traj_vec[i + 1]);
    }

    int n = totg_trajectory.joint_trajectory.points.size();
    std::cout << "moveit trajectory size : " << n << std::endl;
    double timeval = totg_trajectory.joint_trajectory.points[n - 1].time_from_start.toSec();
    double avg_speed = distance / timeval;
    double required_speed = 0.45;
    // double required_speed = 0.20;
    double total_req_time = distance / required_speed;
    double ratio_bt_req_and_curr_time = total_req_time / timeval;
    double required_sampling_time = total_req_time / n;
    std::cout << "distance : " << distance << std::endl;
    std::cout << "total_time_taken : " << timeval << std::endl;
    std::cout << "average speed : " << avg_speed << std::endl;
    std::cout << "total required time : " << total_req_time << std::endl;
    std::cout << "ratio between required and current time : " << ratio_bt_req_and_curr_time << std::endl;

    totg_trajectory_time_scaled_output = applyTimeScaling(totg_trajectory, ratio_bt_req_and_curr_time, n);

    int n1 = totg_trajectory_time_scaled_output.joint_trajectory.points.size();
    std::cout << "totg_time_scaled_size : " << n1 << std::endl;

    std::cout << "--------------------------------------------After time scaled---------------------------------------------------------------" << std::endl;
    std::ofstream myfile1("AfterTimeScaling.txt");
    if (myfile1.is_open())
    {
      myfile1 << totg_trajectory_time_scaled_output;

      myfile1.close();
    }

    //applyTimeOptimalTrajectoryGeneration(move_group_interface, "manipulator", required_sampling_time);

    std::cout << "--------------------------------------------After TOTG time parameterization---------------------------------------------------------------" << std::endl;
    std::ofstream myfile2("AfterTOTGtimeparameterization.txt");
    if (myfile2.is_open())
    {
      myfile2 << totg_trajectory_continuous;

      myfile2.close();
    }

    int size_traj = totg_trajectory_time_scaled_output.joint_trajectory.points.size();

    ros::param::set("IK_STATUS", "COMPLETE");
    totg_trajectory_continuous.joint_trajectory.joint_names = joint_names;
    totg_trajectory.joint_trajectory.joint_names = joint_names;
    totg_trajectory_time_scaled_output.joint_trajectory.joint_names = joint_names;
    //move_group_interface.execute(totg_trajectory_time_scaled_output);
    ROS_INFO("Robot starts to move");
    // executeTrajectory(totg_trajectory_continuous.joint_trajectory);
    calculation_done = true;
    received = false;
    entered = true;
    update_Processing_coat_done = true;
  }
  if (entered)
  {
    final_trajectory_pub.publish(totg_trajectory_continuous);
    final_trajectoryTS_pub.publish(totg_trajectory_time_scaled_output);
    final_trajectoryTO_pub.publish(totg_trajectory_continuous);
  }
  }
  rate.sleep();
   
  ros::waitForShutdown();

  return 0;
}