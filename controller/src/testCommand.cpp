#include "std_msgs/String.h"
#include <sstream>
#include <cstring>
#include <string>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/utils.h>
/*
A node to simulate input into the NavigationNode, PickandPlaceNode and ArmNavigationNode to check their functionality 
*/


int main(int argc, char **argv){
	ros::init(argc, argv, "command");
	ros::NodeHandle n;
	ros::Publisher pickandplacePublisher;
	ros::Publisher movePublisher;
	ros::Publisher armPublisher; 
	pickandplacePublisher = n.advertise<std_msgs::String>("pickandplace", 1000);
	movePublisher = n.advertise<move_base_msgs::MoveBaseGoal>("NavigationTopic", 1000); 
	armPublisher = n.advertise<arm_navigation_msgs::MoveArmGoal>("ArmNavigation", 1000); 
	
	
	//base goal definition
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.pose.orientation.z = 0;
	goal.target_pose.header.frame_id = "base_link"; 
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x = 3;
	goal.target_pose.pose.position.y = 0;
	goal.target_pose.pose.position.z = 0;
	goal.target_pose.pose.orientation.w = 1;

	//pick and place definition
	std_msgs::String grab; 
	grab.data = "pickup"; 

	//arm goal definition starts here
	arm_navigation_msgs::MoveArmGoal armgoal;
  	std::vector<std::string> names(7);
  	names[0] = "r_shoulder_pan_joint";
  	names[1] = "r_shoulder_lift_joint";
  	names[2] = "r_upper_arm_roll_joint";
  	names[3] = "r_elbow_flex_joint";
  	names[4] = "r_forearm_roll_joint";
  	names[5] = "r_wrist_flex_joint";
  	names[6] = "r_wrist_roll_joint";
	armgoal.motion_plan_request.group_name = "right_arm";
	armgoal.motion_plan_request.num_planning_attempts = 1;
	armgoal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);
	armgoal.motion_plan_request.planner_id= std::string("");
	armgoal.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
	armgoal.motion_plan_request.goal_constraints.joint_constraints.resize(names.size());
  	for (unsigned int i = 0 ; i < armgoal.motion_plan_request.goal_constraints.joint_constraints.size(); ++i)
  	{
    		armgoal.motion_plan_request.goal_constraints.joint_constraints[i].joint_name = names[i];
    		armgoal.motion_plan_request.goal_constraints.joint_constraints[i].position = 0.0;
    		armgoal.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.1;
    		armgoal.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.1;
  	}
	armgoal.motion_plan_request.goal_constraints.joint_constraints[0].position = -1.0; //shoulder  horizontal rotation (0->2)? 
  	armgoal.motion_plan_request.goal_constraints.joint_constraints[3].position = -0.7;//elbow vertical rotation
  	armgoal.motion_plan_request.goal_constraints.joint_constraints[5].position = -1; //(0.15 -> 1)?
	

	// arm pose definition starts here 
	arm_navigation_msgs::MoveArmGoal armpose;
  	armpose.motion_plan_request.group_name = "right_arm";
  	armpose.motion_plan_request.num_planning_attempts = 1;
  	armpose.motion_plan_request.planner_id = std::string("");
  	armpose.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  	armpose.motion_plan_request.allowed_planning_time = ros::Duration(5.0);
  
  	arm_navigation_msgs::SimplePoseConstraint desired_pose;
  	desired_pose.header.frame_id = "torso_lift_link";
  	desired_pose.link_name = "r_wrist_roll_link";
  	desired_pose.pose.position.x = 0.3;
  	desired_pose.pose.position.y = -0.188;
  	desired_pose.pose.position.z = 0;

  	desired_pose.pose.orientation.x = 0.0;
  	desired_pose.pose.orientation.y = 0.0;
  	desired_pose.pose.orientation.z = 0.0;
  	desired_pose.pose.orientation.w = 1.0;

  	desired_pose.absolute_position_tolerance.x = 0.02;
  	desired_pose.absolute_position_tolerance.y = 0.02;
  	desired_pose.absolute_position_tolerance.z = 0.02;

  	desired_pose.absolute_roll_tolerance = 0.04;
  	desired_pose.absolute_pitch_tolerance = 0.04;
  	desired_pose.absolute_yaw_tolerance = 0.04;
	arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_pose,armpose);


	//uncomment one of the below, don't try to run more than one at once 
	bool done = false; 
	while(!done){
		armPublisher.publish(armpose);
	sleep(2);
	}
	//pickandplacePublisher.publish(grab);
	//movePublisher.publish(goal);



  
    return 0;
  }
