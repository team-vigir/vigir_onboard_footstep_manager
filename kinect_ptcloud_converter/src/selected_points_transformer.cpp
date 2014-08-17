#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>

using std::cout;
using std::cin;
using std::endl;
using std::vector;
using std::string;

class PointcloudTransformer{
public:
	PointcloudTransformer();
	PointcloudTransformer(string end_frame);

	void transform_cloud(const sensor_msgs::PointCloud2::ConstPtr& msg);

private:
	void perform_transform(sensor_msgs::PointCloud& temp_cloud, sensor_msgs::PointCloud& converted_temp_cloud);
	ros::NodeHandle nh;

	string end_frame;
	ros::Subscriber rviz_plugin_ptcloud_listener;
	ros::Publisher rviz_plugin_selected_pts_transform;
	tf::TransformListener camera_to_kinect_transform;
};

int main(int argc, char** argv){
	ros::init(argc, argv, "selected_points_transformer");
	ros::NodeHandle nh;

	PointcloudTransformer transformer("/adept_combined");

	ros::spin();
}

PointcloudTransformer::PointcloudTransformer()
{
	cout << "Error, default PointcloudTransformer constructor called!" << endl;
	exit(1);
}

PointcloudTransformer::PointcloudTransformer(string end_frame)
:camera_to_kinect_transform(nh)
{
	cout << "Transforming pointclouds to reference frame: " << end_frame << endl;
	this->end_frame = end_frame;
	rviz_plugin_selected_pts_transform = nh.advertise<sensor_msgs::PointCloud2>("/selected_points/transformed", 5);
	rviz_plugin_ptcloud_listener = nh.subscribe("/selected_points", 5, &PointcloudTransformer::transform_cloud, this);
}

void PointcloudTransformer::transform_cloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	cout << "Got a pointcloud to transform." << endl;
	string new_frame = "camera_rgb_optical_frame";
	cout << "Header:" << endl << msg->header;
	cout << "Resetting header frame_id to: " << new_frame << endl;
	sensor_msgs::PointCloud2 in_cloud = *msg;
	sensor_msgs::PointCloud2 out_cloud;
	sensor_msgs::PointCloud temp_cloud;
	sensor_msgs::PointCloud converted_temp_cloud;

	in_cloud.header.frame_id = new_frame;
	sensor_msgs::convertPointCloud2ToPointCloud(in_cloud, temp_cloud);
	perform_transform(temp_cloud, converted_temp_cloud);
	sensor_msgs::convertPointCloudToPointCloud2(converted_temp_cloud, out_cloud);

	rviz_plugin_selected_pts_transform.publish(out_cloud);
}

void PointcloudTransformer::perform_transform(sensor_msgs::PointCloud& temp_cloud, sensor_msgs::PointCloud& converted_temp_cloud)
{
	ros::Duration wait_period(0.1);
	while(1){
		try{
			ros::Time now = ros::Time::now();
			camera_to_kinect_transform.waitForTransform(end_frame, temp_cloud.header.frame_id, now, ros::Duration(0.5));
			camera_to_kinect_transform.transformPointCloud(end_frame, temp_cloud, converted_temp_cloud);
			break;
		
		} catch (...){
			cout << "tf exception caused in perform_transform." << endl;
			wait_period.sleep();
		}
	}
}