#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point.h>

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

struct Box{
	geometry_msgs::Point32 pt1;
	geometry_msgs::Point32 pt2;
};

class KinectAdapter{
	public:
		KinectAdapter();
		KinectAdapter(string fixed_frame, string camera_frame);

		void set_kinect_ptcloud(const sensor_msgs::PointCloud2::ConstPtr& msg);
		void select_point_cloud(Box& bounding_box);

	private:
		void get_pointcloud();
		void change_pointcloud_reference_frame();
		void filter_inliers(Box bounding_box);

		ros::NodeHandle nh;
		string fixed_frame;
		string camera_frame;
		sensor_msgs::PointCloud2 raw_cloud;
		sensor_msgs::PointCloud current_cloud;
		ros::Publisher ptcloud_out;
		tf::TransformListener camera_to_kinect_transform;
};

bool value_is_between_two_limits(double value, double limit1, double limit2);
bool pt_inside_box(Box bounding_box, const geometry_msgs::Point32& pt);
Box get_bounding_box_from_user();
geometry_msgs::Point32 get_pt_from_user(string msg);
void recv_kinect_ptcloud(const sensor_msgs::PointCloud2::ConstPtr& msg);

KinectAdapter *primary_adapter;

int main(int argc, char** argv){
	ros::init(argc, argv, "kinect_ptcloud_node");
	ros::NodeHandle nh;
	string kinect_input_stream_name = "/camera/depth_registered/points";
	string tf_world_frame = "/adept_combined";
	string camera_base_frame = "/camera_link";

	primary_adapter = new KinectAdapter(tf_world_frame, camera_base_frame);
	ros::Subscriber kinect_data_stream = nh.subscribe(kinect_input_stream_name.c_str(),
													 1, recv_kinect_ptcloud);

	ros::AsyncSpinner spinny(1);
	spinny.start();

	Box bounding_box;
	while(ros::ok()){
		bounding_box = get_bounding_box_from_user();
		primary_adapter->select_point_cloud(bounding_box);
	}
}

Box get_bounding_box_from_user()
{
	Box ret_box;
	ret_box.pt1 = get_pt_from_user("Please give one corner of a box (e.g. '3.5 4 5'): ");
	ret_box.pt2 = get_pt_from_user("Please give the second corner of a box (e.g. '1 2 3'): ");

	return ret_box;
}

geometry_msgs::Point32 get_pt_from_user(string msg)
{
	geometry_msgs::Point32 user_input_pt;
	cout << msg << endl;
	cin >> user_input_pt.x;
	cin >> user_input_pt.y;
	cin >> user_input_pt.z;

	return user_input_pt;
}

void recv_kinect_ptcloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	primary_adapter->set_kinect_ptcloud(msg);
	//cout << "Got cloud!" << endl;
}

bool value_is_between_two_limits(double value, double limit1, double limit2)
{
	if (limit1 < limit2){
		if (value >= limit1 && value <= limit2){
			return true;
		}

	} else {
		if (value >= limit2 && value <= limit1){
			return true;
		}

	}

	return false;
}

bool pt_inside_box(Box bounding_box, const geometry_msgs::Point32& pt)
{
	if (value_is_between_two_limits(pt.x, bounding_box.pt1.x, bounding_box.pt2.x)
		&& value_is_between_two_limits(pt.y, bounding_box.pt1.y, bounding_box.pt2.y)
		&& value_is_between_two_limits(pt.z, bounding_box.pt1.z, bounding_box.pt2.z)){
		return true;
	}

	return false;
}

KinectAdapter::KinectAdapter()
{
	cout << "Default KinectAdapter constructor called. Error!" << endl;
	exit(1);
}

KinectAdapter::KinectAdapter(string fixed_frame, string camera_frame)
:camera_to_kinect_transform(nh)
{
	this->fixed_frame = fixed_frame;
	this->camera_frame = camera_frame;
	ptcloud_out = nh.advertise<sensor_msgs::PointCloud2>("kinect/selected_cloud", 1);
}

void KinectAdapter::set_kinect_ptcloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	//cout << "Cloud received." << endl;
	raw_cloud = *msg;
}

void KinectAdapter::select_point_cloud(Box& bounding_box)
{
	get_pointcloud();
	change_pointcloud_reference_frame();
	filter_inliers(bounding_box);
	
	sensor_msgs::PointCloud2 out_cloud;
	sensor_msgs::convertPointCloudToPointCloud2(current_cloud, out_cloud);
	ptcloud_out.publish(out_cloud);
	cout << "Size of published cloud: " << current_cloud.points.size() << endl;
	cout << "Cloud Header: " << current_cloud.header << endl;
}

void KinectAdapter::get_pointcloud()
{
	//bool success = 
	sensor_msgs::convertPointCloud2ToPointCloud(raw_cloud, current_cloud);
	/*if(success){
		cout << "Conversion to pointcloud v1 successful." << endl;
	} else {
		cout << "Conversion to pointcloud v1 failed." << endl; 
	}*/

}

void KinectAdapter::change_pointcloud_reference_frame()
{
	sensor_msgs::PointCloud temp;
	camera_to_kinect_transform.transformPointCloud(fixed_frame, current_cloud, temp);

	current_cloud = temp;
}

void KinectAdapter::filter_inliers(Box bounding_box)
{
	vector<geometry_msgs::Point32> selected_points;
	long num_pts = current_cloud.points.size();
	for (long i = 0; i < num_pts; ++i){
		if (pt_inside_box(bounding_box, current_cloud.points[i])){
			selected_points.push_back(current_cloud.points[i]);
			
		}

	}

	current_cloud.points = selected_points;
}
