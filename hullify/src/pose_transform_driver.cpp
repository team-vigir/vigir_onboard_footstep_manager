#include "pose_transform.h"

int main(int argc, char** argv){
	ros::init(argc, argv, "pose_playing_node");
	ros::NodeHandle n;

	ros::Publisher pose_topic = n.advertise<geometry_msgs::PoseStamped>("pose_playing/results", 1);
	ros::Publisher initial_pose_topic = n.advertise<geometry_msgs::PoseStamped>("pose_playing/initial", 5);

	geometry_msgs::PoseStamped pose_template;
	pose_template.header.frame_id = "world";
	pose_template.header.stamp = ros::Time::now();

	geometry_msgs::Point start_pt;
	start_pt.x = 0;
	start_pt.y = 0;
	start_pt.z = 0;

	pose_template.pose.position = start_pt;

	geometry_msgs::Quaternion quat;
	quat.w = 1;
	quat.x = 0;
	quat.y = 0;
	quat.z = 0;

	pose_template.pose.orientation = quat;

	initial_pose_topic.publish(pose_template);

	geometry_msgs::PoseStamped new_pose = pose_template;
	quat.w = 0;
	quat.x = 0;
	quat.y = 0;
	quat.z = 2;
	new_pose.pose.orientation = quat;
	start_pt.x = 1;
	start_pt.y = 1;
	start_pt.z = 1;
	new_pose.pose.position = start_pt;	
	ros::Duration period(2);
	
	Eigen::Vector3d z_axis(0, 0, 1), x_axis(1, 0, 0), y_axis(0, 1, 0);
	Axes reference_axes, user_axes;
	reference_axes.x_axis = x_axis;
	reference_axes.y_axis = y_axis;
	reference_axes.z_axis = z_axis;

	Eigen::Quaterniond transform;
	while(ros::ok()){
		cout << "Please input a z axis: ";
		cin >> z_axis[0] >> z_axis[1] >> z_axis[2];
		cout << "Please input an x axis: ";
		cin >> x_axis[0] >> x_axis[1] >> x_axis[2];

		user_axes.x_axis = x_axis;
		user_axes.z_axis = z_axis;
		transform = get_axes_transformation(reference_axes, user_axes);

		quat.w = transform.w();
		quat.x = transform.x();
		quat.y = transform.y();
		quat.z = transform.z();
		new_pose.pose.orientation = quat;
		new_pose.header.stamp = ros::Time::now();

		initial_pose_topic.publish(pose_template);
		pose_topic.publish(new_pose);
		period.sleep();
	}

	return 0;
}
