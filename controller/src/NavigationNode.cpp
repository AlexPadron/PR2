#include "std_msgs/String.h"
#include <sstream>
#include <cstring>
#include <string>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
/*
takes input of move_base_msgs::MoveBaseGoal type on topic NavigationTopic and
publishes it to the robot through the action client 
*/
//variables defined in main,  
move_base_msgs::MoveBaseGoal goal;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
MoveBaseClient * a = NULL; 
int count = 0; 
 
//the trick is to only make and use the action client within this method since it is private
void processInput(const move_base_msgs::MoveBaseGoal inputGoal){
	ROS_INFO("Recieved a goal"); 
	MoveBaseClient ac("move_base", true);
	while(!ac.waitForServer(ros::Duration(5.0)))
    	ROS_INFO("Waiting for the move_base action server to come up");
 		
	
	ROS_INFO("Sending goal"); 
	ac.sendGoal(inputGoal);
	ac.waitForResult();
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    		ROS_INFO("Moved Sucessfully");
  	else
    		ROS_INFO("Move failed");  
	count = count +1; 
}




//main function, listens for input of NavigationTopic and published to robot 
int main(int argc, char **argv){
	//set up a bunch of variables 
	ros::init(argc, argv, "NavigationNode");
	ros::NodeHandle n;
	ROS_INFO("Start Subscribing"); 
	ros::Subscriber input = n.subscribe("NavigationTopic", 100, processInput);
	ros::spin();
	return 0; 
}
