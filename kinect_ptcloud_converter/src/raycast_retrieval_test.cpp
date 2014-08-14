/*********************************************
* The purpose of this file is to see if the worldmodel is reading
* 		kinect data as it is published and can return such data
*************************************************/
#include "ros/ros.h"
#include <geometry_msgs/Point.h>
#include <flor_perception_msgs/RaycastRequest.h>

#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>

using std::cout;
using std::cin;
using std::endl;
using std::vector;
using std::string;

int main(int argc, char** argv){
	ros::init(argc, argv, "raycast_retrieval_test");
	ros::NodeHandle nh;

	ros::Publisher raycast_requester = nh.advertise<flor_perception_msgs::RaycastRequest>("/flor/worldmodel/ocs/dist_query_pointcloud_request_world", 1);

	flor_perception_msgs::RaycastRequest req;
	req.origin.x = -10;
	req.origin.y = -10;
	req.origin.z = -10;

	req.direction.x = 40;
	req.direction.y = 40;
	req.direction.z = 40;

	ros::Duration(1);
	while(ros::ok()){
		raycast_requester.publish(req);
	}

	return 0;
}