#include <move_arm/move_arm.h>

MoveArm::MoveArm(ros::NodeHandle* nh):nh_(nh),arm_motion_commander(nh)
{
}

bool MoveArm::setPrepose(){
    arm_motion_commander.plan_move_to_pre_pose();
    //send command to execute planned motion
    arm_motion_commander.rt_arm_execute_planned_path();

return true;
}

bool MoveArm::graspCan(Eigen::Vector3f centroid){
	geometry_msgs::PoseStamped rt_tool_pose;
        rt_tool_pose.header.frame_id = "torso";
	rt_tool_pose.pose.position.x=centroid[0];
	rt_tool_pose.pose.position.y=centroid[1];
	rt_tool_pose.pose.position.z=centroid[2];
    	rt_tool_pose.pose.orientation.x = 0.166;
    	rt_tool_pose.pose.orientation.y = 0.648;
    	rt_tool_pose.pose.orientation.z = 0.702;
    	rt_tool_pose.pose.orientation.w = 0.109;
    	arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(rt_tool_pose);
    //send command to execute planned motion
    	arm_motion_commander.rt_arm_execute_planned_path();
return true;
}

bool MoveArm::moveCan(){
	Eigen::Vector3d dp_displacement;
	int rtn_val;
	dp_displacement<<0,0,0.3;
	rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_dp_xyz(dp_displacement);
    	if (rtn_val == cartesian_planner::baxter_cart_moveResult::SUCCESS)  { 
         	   //send command to execute planned motion
         	  rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
    	}
return true;
}

bool MoveArm::dropCan(){

return true;
}
