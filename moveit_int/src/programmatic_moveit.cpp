#include <ros/ros.h>
#include "osu_ros_adept/robot_movement_command.h"

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <iostream>
#include <string>
#include <vector>

using std::cout;
using std::cin;
using std::endl;
using std::string;
using std::vector;

struct AdeptJoint{
	string name;
	double position;
};

class AdeptInterface{
public:
	AdeptInterface();
	~AdeptInterface();

	void move_hand(const geometry_msgs::PoseStamped::ConstPtr& msg);

private:
	void init_mk_movement_msg_template();
	void set_hand_goal(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void display_arm_trajectory();

	ros::NodeHandle node_handle;
	moveit::planning_interface::MoveGroup group;
	moveit::planning_interface::MoveGroup::Plan adept_plan;
	AdeptJoint* robot_joints;
	osu_ros_adept::robot_movement_command movement_msg;

	ros::Publisher display_publisher;
	ros::ServiceClient action_request;
	moveit_msgs::DisplayTrajectory display_trajectory;

};

//void print_end_state(moveit_msgs::DisplayTrajectory& the_plan);
void print_joint_state(moveit::planning_interface::MoveGroup& joint_group);
void get_end_state(moveit_msgs::DisplayTrajectory& the_plan, AdeptJoint* robot_joints, osu_ros_adept::robot_movement_command& movement_msg);


int main(int argc, char **argv){
 	ros::init(argc, argv, "programmatic_moveit");
  	ros::NodeHandle node_handle;
  	ros::AsyncSpinner spinner(1);
  	spinner.start();

	AdeptInterface adept;
	ros::Subscriber position_info = node_handle.subscribe("adept_wrist_orientation", 1, &AdeptInterface::move_hand, &adept);

	while(ros::ok());

	return 0;
}

AdeptInterface::AdeptInterface()
:group("adept_arm")
{
	display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/jc_planned_path", 1, true);
	action_request = node_handle.serviceClient<osu_ros_adept::robot_movement_command>("send_robot_movements");
	
	cout << "Reference frame: " << group.getPlanningFrame() << endl;

	robot_joints = new AdeptJoint[6];

}

AdeptInterface::~AdeptInterface()
{
	delete [] robot_joints;
}

void AdeptInterface::init_mk_movement_msg_template()
{
	movement_msg.request.length = 60;
	movement_msg.request.type = 1; //1 = angle_msg
	movement_msg.request.command = 0;
	movement_msg.request.reply = 0;
	movement_msg.request.unused = 0;	
}

void AdeptInterface::move_hand(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	set_hand_goal(msg);

	bool victory = group.plan(adept_plan);
	if (victory){
		cout << "Planning registered. New end effector positions:" << endl;
		print_joint_state(group);
		display_arm_trajectory();
		
		//const robot_state::RobotState state = group.getJointValueTarget();
		//state.printStatePositions();
		get_end_state(display_trajectory, robot_joints, movement_msg);

		action_request.call(movement_msg);
		cout << "Bytes sent: " << movement_msg.response.bytes_sent << endl;

	} else {
		ROS_ERROR("No suitable motion plan found.");

	}
}

void AdeptInterface::set_hand_goal(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	group.setPositionTarget(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
	cout << "programmatic_moveit read position: x- " << msg->pose.position.x 
		<< " y- " << msg->pose.position.y << " z- " << msg->pose.position.z << endl;
	
	group.setOrientationTarget(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
}

void print_joint_state(moveit::planning_interface::MoveGroup& joint_group)
{
	vector<string> joint_names = joint_group.getJoints();
	vector<double> joint_positions = joint_group.getCurrentJointValues();

	cout << "Current joint values:" << endl;
	long num_joints = joint_names.size();
	for (long i = 0; i < num_joints; ++i){
		cout << "\t" << joint_names[i] << " has joint angle: " << joint_positions[i] << endl;
	}
}

void AdeptInterface::display_arm_trajectory()
{
	display_trajectory.trajectory_start = adept_plan.start_state_;
	display_trajectory.trajectory.push_back(adept_plan.trajectory_);
	display_publisher.publish(display_trajectory);
}

void get_end_state(moveit_msgs::DisplayTrajectory& the_plan, AdeptJoint* robot_joints, osu_ros_adept::robot_movement_command& movement_msg)
{
	int final_traj_idx = the_plan.trajectory.size() - 1;
	long final_pos_idx = the_plan.trajectory[final_traj_idx].joint_trajectory.points.size() - 1;
	int num_joints = the_plan.trajectory[final_traj_idx].joint_trajectory.points[1].positions.size();

	trajectory_msgs::JointTrajectory* traj = &(the_plan.trajectory[final_traj_idx].joint_trajectory);
	for (int i = 0; i < num_joints; ++i){
		robot_joints[i].name = traj->joint_names[i]; 
		robot_joints[i].position = traj->points[final_pos_idx].positions[i];
		movement_msg.request.jts[i] = traj->points[final_pos_idx].positions[i];
	}
}
