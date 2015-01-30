#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <cstring>
#include <string>
#include <actionlib/client/simple_action_client.h>
#include <tabletop_object_detector/TabletopDetection.h>
#include <tabletop_collision_map_processing/TabletopCollisionMapProcessing.h>
#include <object_manipulation_msgs/PickupAction.h>
#include <object_manipulation_msgs/PlaceAction.h>
/*
Takes input on "pickandplace" topic and uses the action client to command the robot
Currently there is a glitch in the place method I found, so this code only has the 
pick up method, which works well. I will put in the place method once I get it working
*/
ros::NodeHandle * n = NULL; 
std::string message;
//has to be void, this took me forever to figure out 
//make sure to have a pointer as input (current syntax works fine) 
void processInput(const std_msgs::String::ConstPtr& msg){
	ROS_INFO("Recieved a message"); 
	ros::NodeHandle nh = *n; 
	message = msg->data; 
	

	const std::string OBJECT_DETECTION_SERVICE_NAME = "/object_detection";
	const std::string COLLISION_PROCESSING_SERVICE_NAME = "/tabletop_collision_map_processing/tabletop_collision_map_processing";
	const std::string PICKUP_ACTION_NAME = "/object_manipulator/object_manipulator_pickup";
	const std::string PLACE_ACTION_NAME = "/object_manipulator/object_manipulator_place";

	ros::ServiceClient object_detection_srv;
	ros::ServiceClient collision_processing_srv;
	actionlib::SimpleActionClient<object_manipulation_msgs::PickupAction> pickup_client(PICKUP_ACTION_NAME, true);
	actionlib::SimpleActionClient<object_manipulation_msgs::PlaceAction> place_client(PLACE_ACTION_NAME, true);

	//wait for detection client
  	while ( !ros::service::waitForService(OBJECT_DETECTION_SERVICE_NAME, ros::Duration(2.0)) && nh.ok() ) 
  	{
    		ROS_INFO("Waiting for object detection service to come up");
  	}
  	if (!nh.ok()) exit(0);

  	object_detection_srv = nh.serviceClient<tabletop_object_detector::TabletopDetection>
    	(OBJECT_DETECTION_SERVICE_NAME, true);
		
	//wait for collision map processing client
  	while ( !ros::service::waitForService(COLLISION_PROCESSING_SERVICE_NAME, ros::Duration(2.0)) && nh.ok() ) 
  	{
    		ROS_INFO("Waiting for collision processing service to come up");
  	}

  	if (!nh.ok()) exit(0);
  	collision_processing_srv = nh.serviceClient<tabletop_collision_map_processing::TabletopCollisionMapProcessing> 				(COLLISION_PROCESSING_SERVICE_NAME, true);
	//wait for pickup client
  	while(!pickup_client.waitForServer(ros::Duration(2.0)) && nh.ok())
  	{
    		ROS_INFO_STREAM("Waiting for action client " << PICKUP_ACTION_NAME);
  	}
  	if (!nh.ok()) exit(0);
   	ros::Subscriber sub = nh.subscribe("pickandplace", 1000, processInput);
	
	if(message == "pickup"){
		//call the tabletop detection
  		ROS_INFO("Calling tabletop detector");
  		tabletop_object_detector::TabletopDetection detection_call;
  		//we want recognized database objects returned
  		//set this to false if you are using the pipeline without the database
  		detection_call.request.return_clusters = true;
  		//we want the individual object point clouds returned as well
  		detection_call.request.return_models = true;
  		detection_call.request.num_models = 1;
  		if (!object_detection_srv.call(detection_call))
  		{
    		ROS_ERROR("Tabletop detection service failed");	
  		}
  		if (detection_call.response.detection.result != detection_call.response.detection.SUCCESS)
  		{
    			ROS_ERROR("Tabletop detection returned error code %d", detection_call.response.detection.result);
    			
  		}
  		if (detection_call.response.detection.clusters.empty() && detection_call.response.detection.models.empty() )
  		{
    			ROS_ERROR("The tabletop detector detected the table, but found no objects");
  		}

		//call collision map processing
  		ROS_INFO("Calling collision map processing");
  		tabletop_collision_map_processing::TabletopCollisionMapProcessing processing_call;
  		//pass the result of the tabletop detection 
  		processing_call.request.detection_result = detection_call.response.detection;
  		//ask for the existing map and collision models to be reset
  		processing_call.request.reset_collision_models = true;
  		processing_call.request.reset_attached_models = true;
  		//ask for the results to be returned in base link frame
  		processing_call.request.desired_frame = "base_link";
  		if (!collision_processing_srv.call(processing_call))
  		{
    			ROS_ERROR("Collision map processing service failed");
  		}
  		//the collision map processor returns instances of graspable objects
  		if (processing_call.response.graspable_objects.empty())
  		{
    			ROS_ERROR("Collision map processing returned no graspable objects");
  		}
		//call object pickup
  		ROS_INFO("Calling the pickup action");
  		object_manipulation_msgs::PickupGoal pickup_goal;
  		//pass one of the graspable objects returned 
  		//by the collision map processor
  		pickup_goal.target = processing_call.response.graspable_objects.at(0);
  		//pass the name that the object has in the collision environment
  		//this name was also returned by the collision map processor
  		pickup_goal.collision_object_name = processing_call.response.collision_object_names.at(0);
  		//pass the collision name of the table, also returned by the collision 
  		//map processor
  		pickup_goal.collision_support_surface_name = processing_call.response.collision_support_surface_name;
  		//pick up the object with the right arm
  		pickup_goal.arm_name = "right_arm";
  		//we will be lifting the object along the "vertical" direction
  		//which is along the z axis in the base_link frame
  		geometry_msgs::Vector3Stamped direction;
  		direction.header.stamp = ros::Time::now();
  		direction.header.frame_id = "base_link";
  		direction.vector.x = 0;
  		direction.vector.y = 0;
  		direction.vector.z = 1;
  		pickup_goal.lift.direction = direction;
  		//request a vertical lift of 10cm after grasping the object
  		pickup_goal.lift.desired_distance = 0.1;
  		pickup_goal.lift.min_distance = 0.05;
  		//do not use tactile-based grasping or tactile-based lift
  		pickup_goal.use_reactive_lift = false;
  		pickup_goal.use_reactive_execution = false;
  		//send the goal
  		pickup_client.sendGoal(pickup_goal);
  		while (!pickup_client.waitForResult(ros::Duration(10.0)))
  		{
    			ROS_INFO("Waiting for the pickup action...");
  		}
  		object_manipulation_msgs::PickupResult pickup_result = *(pickup_client.getResult());
  		if (pickup_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
  		{
    			ROS_ERROR("The pickup action has failed with result code %d", pickup_result.manipulation_result.value);
  		}
	}
	if(message == "place"){
		//TODO put place method here 
	}



}
	
int main(int argc, char **argv){
	ros::init(argc, argv, "PickandPlaceNode");
	ros::NodeHandle nh;
	n = &nh; 
	bool done = false;  
	while(!done){
	ros::Subscriber sub = nh.subscribe("pickandplace", 1000, processInput); 
	ROS_INFO("subscriber called");
   	ros::spin();
	}


  
    return 0;
  }
