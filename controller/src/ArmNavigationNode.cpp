#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <arm_navigation_msgs/MoveArmAction.h>
/*
This node controls the arms of the robot with arm_navigation_msgs::MoveArmGoal messages.
This can be used to either specify joint positions or a target hand location.
*/

ros::NodeHandle * n = NULL;
int count = 0; 

void processInput(arm_navigation_msgs::MoveArmGoal goal){
	ROS_INFO("Recieved a message"); 
	ros::NodeHandle nh = * n; 
		actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm("move_right_arm",true);
  		move_arm.waitForServer();
  		ROS_INFO("Connected to server");	 
  		std::vector<std::string> names(7);

	if (nh.ok())
  	{
   		bool finished_within_time = false;
    		move_arm.sendGoal(goal);
    		finished_within_time = move_arm.waitForResult(ros::Duration(200.0));
    		if (!finished_within_time)
    		{
     			move_arm.cancelGoal();
      			ROS_INFO("Timed out achieving goal A");
    		}
    		else
    		{	
      			actionlib::SimpleClientGoalState state = move_arm.getState();
      			bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
      			if(success)
        			ROS_INFO("Action finished: %s",state.toString().c_str());
      			else
        			ROS_INFO("Action failed: %s",state.toString().c_str());
    		}
  	}
}




int main(int argc, char **argv){
	ros::init (argc, argv, "MoveArmNode");
	ros::NodeHandle nh;
	n = &nh; 
	ROS_INFO("Start Subscribing"); 
	ros::Subscriber input = nh.subscribe("ArmNavigation", 100, processInput);
	ros::spin();
	return 0; 

}
