#include "ros/ros.h"
#include "status_check/GetProcessStatus.h"
#include "status_check/ProcessFlag.h"
#include "status_check/ProcessStatus.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "status_check/Status_data.h"
#include "status_check/stats.h"
#include "status_check/GetProcessStatus.h"


void CheckStatus(ros::NodeHandle *n);

status_check::GetProcessStatus::Response current_process_status;

bool GetStatus(status_check::GetProcessStatus::Request  &req, status_check::GetProcessStatus::Response &res){
    

  res.current_process_status = current_process_status.current_process_status;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_process_status_server");
  ros::NodeHandle n;
  CheckStatus(&n);
  ros::ServiceServer service = n.advertiseService("get_process_status", GetStatus);
  ROS_INFO("Ready to monitor the process...");

  ros::Rate loop_rate(1000);
  while (ros::ok()){
    
    CheckStatus(&n);
    ros::spinOnce();
    loop_rate.sleep();

  }  
  return 0;
}

void CheckStatus(ros::NodeHandle *nh){
  
    if(nh->hasParam("axalta/ccscore/dashboard/PAINTJOBPROCESS")){
      bool val;
      nh->getParam("axalta/ccscore/dashboard/PAINTJOBPROCESS",val);
      current_process_status.current_process_status.path_planning.process_ready = true;
      current_process_status.current_process_status.path_planning.process_active = val;
      current_process_status.current_process_status.path_planning.process_completed = !val;
      current_process_status.current_process_status.path_planning.process_error = false;
    }
    if(nh->hasParam("axalta/ccscore/arm_service/BASE_COAT_1")){
      bool val;
      nh->getParam("axalta/ccscore/arm_service/BASE_COAT_1",val);
      current_process_status.current_process_status.base_coat1.process_ready = true;
      current_process_status.current_process_status.base_coat1.process_active = !val;
      current_process_status.current_process_status.base_coat1.process_completed = val;
      current_process_status.current_process_status.base_coat1.process_error = false;
  
    }
    if(nh->hasParam("axalta/ccscore/arm_service/BASE_COAT_2")){
      bool val;
      nh->getParam("axalta/ccscore/arm_service/BASE_COAT_2",val);
      current_process_status.current_process_status.base_coat2.process_ready = true;
      current_process_status.current_process_status.base_coat2.process_active = !val;
      current_process_status.current_process_status.base_coat2.process_completed = val;
      current_process_status.current_process_status.base_coat2.process_error = false;
  
    }
    if(nh->hasParam("axalta/ccscore/arm_service/SEALER_COAT_1")){
      bool val;
      nh->getParam("axalta/ccscore/arm_service/SEALER_COAT_1",val);
      current_process_status.current_process_status.sealer_coat.process_ready = true;
      current_process_status.current_process_status.sealer_coat.process_active = !val;
      current_process_status.current_process_status.sealer_coat.process_completed = val;
      current_process_status.current_process_status.sealer_coat.process_error = false;
  
    }
    /***if(nh->hasParam("axalta/ccscore/arm_service/SEALER_COAT_2")){
      bool val;
      nh->getParam("axalta/ccscore/arm_service/SEALER_COAT_2",val);
      current_process_status.current_process_status.sealer_coat2.process_ready = true;
      current_process_status.current_process_status.sealer_coat2.process_active = !val;
      current_process_status.current_process_status.sealer_coat2.process_completed = val;
      current_process_status.current_process_status.sealer_coat2.process_error = false;
  
    }****/
    if(nh->hasParam("axalta/ccscore/arm_service/CLEAR_COAT_1")){
      bool val;
      nh->getParam("axalta/ccscore/arm_service/CLEAR_COAT_1",val);
      current_process_status.current_process_status.clear_coat1.process_ready = true;
      current_process_status.current_process_status.clear_coat1.process_active = !val;
      current_process_status.current_process_status.clear_coat1.process_completed = val;
      current_process_status.current_process_status.clear_coat1.process_error = false;
  
    }
    if(nh->hasParam("axalta/ccscore/arm_service/CLEAR_COAT_2")){
      bool val;
      nh->getParam("axalta/ccscore/arm_service/CLEAR_COAT_2",val);
      current_process_status.current_process_status.clear_coat2.process_ready = true;
      current_process_status.current_process_status.clear_coat2.process_active = !val;
      current_process_status.current_process_status.clear_coat2.process_completed = val;
      current_process_status.current_process_status.clear_coat2.process_error = false;
  
    }
    if(nh->hasParam("axalta/ccscore/dashboard/NEWJOB")){
      bool val;
      nh->getParam("axalta/ccscore/dashboard/NEWJOB",val);
      current_process_status.current_process_status.mir_home_to_pos_move.process_ready = true;
      current_process_status.current_process_status.mir_home_to_pos_move.process_active = false;
      current_process_status.current_process_status.mir_home_to_pos_move.process_completed = val;
      current_process_status.current_process_status.mir_home_to_pos_move.process_error = false;
  
    }
    
    if(nh->hasParam("axalta/ccscore/arm_service/mir_pos_to_home_move_first")){
      bool val;
      nh->getParam("axalta/ccscore/arm_service/mir_pos_to_home_move_first",val);
      current_process_status.current_process_status.mir_pos_to_home_move_first.process_ready = true;
      current_process_status.current_process_status.mir_pos_to_home_move_first.process_active = false;
      current_process_status.current_process_status.mir_pos_to_home_move_first.process_completed = val;
      current_process_status.current_process_status.mir_pos_to_home_move_first.process_error = false;
  
    }
    if(nh->hasParam("axalta/ccscore/arm_service/mir_pos_to_home_move_second")){
      bool val;
      nh->getParam("axalta/ccscore/arm_service/mir_pos_to_home_move_second",val);
      current_process_status.current_process_status.mir_pos_to_home_move_second.process_ready = true;
      current_process_status.current_process_status.mir_pos_to_home_move_second.process_active = false;
      current_process_status.current_process_status.mir_pos_to_home_move_second.process_completed = val;
      current_process_status.current_process_status.mir_pos_to_home_move_second.process_error = false;
  
    }
    if(nh->hasParam("axalta/ccscore/arm_service/mir_pos_to_home_move_finish")){
      bool val;
      nh->getParam("axalta/ccscore/arm_service/mir_pos_to_home_move_finish",val);
      current_process_status.current_process_status.mir_pos_to_home_move_finish.process_ready = true;
      current_process_status.current_process_status.mir_pos_to_home_move_finish.process_active = false;
      current_process_status.current_process_status.mir_pos_to_home_move_finish.process_completed = val;
      current_process_status.current_process_status.mir_pos_to_home_move_finish.process_error = false;
  
    }
    if(nh->hasParam("axalta/ccscore/dashboard/mir_home_to_pos_move_second")){
      bool val;
      nh->getParam("axalta/ccscore/dashboard/mir_home_to_pos_move_second",val);
      current_process_status.current_process_status.mir_pos_to_home_move_finish.process_ready = true;
      current_process_status.current_process_status.mir_pos_to_home_move_finish.process_active = false;
      current_process_status.current_process_status.mir_pos_to_home_move_finish.process_completed = val;
      current_process_status.current_process_status.mir_pos_to_home_move_finish.process_error = false;
  
    }
    if(nh->hasParam("axalta/ccscore/dashboard/mir_home_to_pos_move_clear")){
      bool val;
      nh->getParam("axalta/ccscore/dashboard/mir_home_to_pos_move_clear",val);
      current_process_status.current_process_status.mir_pos_to_home_move_finish.process_ready = true;
      current_process_status.current_process_status.mir_pos_to_home_move_finish.process_active = false;
      current_process_status.current_process_status.mir_pos_to_home_move_finish.process_completed = val;
      current_process_status.current_process_status.mir_pos_to_home_move_finish.process_error = false;
  
    }
}
