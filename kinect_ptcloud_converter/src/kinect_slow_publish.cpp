#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>

#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>

using std::cout;
using std::cin;
using std::endl;
using std::vector;
using std::string;

sensor_msgs::PointCloud2 latest_pointcloud;
sensor_msgs::Image latest_depth_image;

bool has_new_data = false;
void set_latest_pointcloud(const sensor_msgs::PointCloud2::ConstPtr& msg);
void set_latest_depth_image(const sensor_msgs::Image::ConstPtr& msg);

int main(int argc, char** argv){
	ros::init(argc, argv, "kinect_slow_publish");
	ros::NodeHandle n;

	ros::Subscriber get_current_cloud = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 5, set_latest_pointcloud);
	ros::Publisher ptcloud_output_topic = n.advertise<sensor_msgs::PointCloud2>("/camera/depth_registered/2hz_playback_pts", 5);
	
	ros::Subscriber get_current_depth = n.subscribe<sensor_msgs::Image>("/camera/depth/image", 5, set_latest_depth_image);
	ros::Publisher depth_output_topic = n.advertise<sensor_msgs::Image>("/camera/depth/2hz_playback_depth", 5);
	
	ros::AsyncSpinner spinny(1);
	spinny.start();

	ros::Duration period(0.3);
	while(ros::ok()){
		if (!has_new_data){
			cout << "No Kinect data yet sent into kinect_slow_publish node." << endl;
		} else {
			ptcloud_output_topic.publish(latest_pointcloud);
			depth_output_topic.publish(latest_depth_image);
		}

		period.sleep();

	}
}

void set_latest_pointcloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	latest_pointcloud = *msg;
	has_new_data = true;
}

void set_latest_depth_image(const sensor_msgs::Image::ConstPtr& msg)
{
	latest_depth_image = *msg;
	has_new_data = true;
}