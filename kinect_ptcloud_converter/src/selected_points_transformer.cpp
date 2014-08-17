#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "tf/tf.h"

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
	void set_selection_density_percentage();

private:
	void perform_transform(sensor_msgs::PointCloud& temp_cloud, sensor_msgs::PointCloud& converted_temp_cloud);
	void filter_points(sensor_msgs::PointCloud& in_cloud);
	void random_selection(sensor_msgs::PointCloud& in_cloud);
	void filter_kinect_field_of_view(sensor_msgs::PointCloud& in_cloud);
	tf::StampedTransform get_kinect_pose();
	ros::NodeHandle nh;

	string end_frame;
	ros::Subscriber rviz_plugin_ptcloud_listener;
	ros::Publisher rviz_plugin_selected_pts_transform;
	tf::TransformListener camera_to_kinect_transform;
	double percentage;
};

int main(int argc, char** argv){
	ros::init(argc, argv, "selected_points_transformer");
	ros::NodeHandle nh;

	PointcloudTransformer transformer("/adept_combined");

	ros::AsyncSpinner spinny(1);
	spinny.start();

	srand(time(NULL));
	while (ros::ok()){
		transformer.set_selection_density_percentage();

	}
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
	percentage = 100;
}

void PointcloudTransformer::set_selection_density_percentage()
{
	cout << "Please input a percentage (50 for half) of points that should pass random filtering: ";
	cin >> percentage;

	if (cin.fail()){
		cout << "Invalid entry, setting percentage to 100." << endl;
		percentage = 100;

		cin.clear();
	}
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
	filter_points(converted_temp_cloud);
	
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

void PointcloudTransformer::filter_points(sensor_msgs::PointCloud& in_cloud)
{
	cout << "Beginning filtering." << endl;
	filter_kinect_field_of_view(in_cloud);
	random_selection(in_cloud);
	cout << "Finished Filtering." << endl;
}

void PointcloudTransformer::filter_kinect_field_of_view(sensor_msgs::PointCloud& in_cloud)
{
	double kinect_angular_view = (80 / 2) * (M_PI / 180);
	tf::StampedTransform kinect_pose = get_kinect_pose();
	tf::Vector3 standard_x_axis(1, 0, 0);
	tf::Vector3 kinect_line_of_sight = quatRotate(kinect_pose.getRotation(), standard_x_axis);
	tf::Vector3 kinect_camera_pos = kinect_pose.getOrigin();

	geometry_msgs::Point32 cur_pt;
	tf::Vector3 cur_vector;
	double cur_angle;
	long num_pts = in_cloud.points.size();
	vector<geometry_msgs::Point32> out_pts;
	for (long i = 0; i < num_pts; ++i){
		cur_pt = in_cloud.points[i];
		cur_vector = tf::Vector3(cur_pt.x - kinect_camera_pos.x(), cur_pt.y - kinect_camera_pos.y(), cur_pt.z - kinect_camera_pos.z());
		cur_angle = tf::tfAngle(kinect_line_of_sight, cur_vector);
		
		if (cur_angle < kinect_angular_view){
			out_pts.push_back(cur_pt);
		}
	}

	cout << num_pts - out_pts.size() << " points were filtered based on the Kinect's viewing angle." << endl;
	in_cloud.points = out_pts;
}

tf::StampedTransform PointcloudTransformer::get_kinect_pose()
{
	ros::Duration wait_period(0.1);
	tf::StampedTransform kinect_transform;
	static string kinect_camera_frame = "/camera_rgb_frame";
	while(1){
		try{
			cout << "Getting transform" << endl;
			ros::Time now = ros::Time::now();
			camera_to_kinect_transform.waitForTransform(end_frame, kinect_camera_frame, now, ros::Duration(0.5));
			camera_to_kinect_transform.lookupTransform(end_frame, kinect_camera_frame, ros::Time(0), kinect_transform);
			cout << "Got transform" << endl;
			break;

		} catch (...){
			ROS_INFO("Could not immediately get kinect transform... Unhandled exception.");
			wait_period.sleep();
		}
	}

	cout << "Kinect transform origin: x-" << kinect_transform.getOrigin().x() << " y-" << kinect_transform.getOrigin().y() << " z-" << kinect_transform.getOrigin().z() << endl;
	tf::Vector3 x_axis(1, 0, 0);
	cout << "Kinect transform x-axis orientation: " << quatRotate(kinect_transform.getRotation(), x_axis) << endl;
	
	return kinect_transform;
}

void PointcloudTransformer::random_selection(sensor_msgs::PointCloud& in_cloud)
{
	cout << "A good optimization would be to include the random_selection() in the field of view filtering..." << endl;
	long num_pts = in_cloud.points.size();
	vector<geometry_msgs::Point32> out_pts;
	int rand_value;
	for (long i = 0; i < num_pts; ++i){
		rand_value = rand() % 100;
		if (percentage > rand_value){
			out_pts.push_back(in_cloud.points[i]);
		}
	}

	cout << num_pts - out_pts.size() << " points were randomly filtered out." << endl;
	in_cloud.points = out_pts;
}