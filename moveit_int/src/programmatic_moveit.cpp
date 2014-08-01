#include <ros/ros.h>
#include "osu_ros_adept/robot_movement_command.h"

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>

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

	void move_hand();
	void recv_new_hand_goal(const geometry_msgs::PoseStamped::ConstPtr& goal);
private:
	void init_mk_movement_msg_template();
	void set_hand_goal();
	void display_arm_trajectory();
	void add_table();
	moveit_msgs::AttachedCollisionObject mk_table();

	ros::NodeHandle node_handle;
	moveit::planning_interface::MoveGroup group;
	moveit::planning_interface::MoveGroup::Plan adept_plan;
	bool has_new_goal;
	geometry_msgs::PoseStamped::Ptr current_goal;
	AdeptJoint* robot_joints;
	osu_ros_adept::robot_movement_command movement_msg;

	ros::Publisher display_publisher;
	ros::Publisher planning_scene_diff_publisher;
	ros::ServiceClient action_request;
	moveit_msgs::DisplayTrajectory display_trajectory;

};

void get_hand_pose_request(const geometry_msgs::PoseStamped::ConstPtr& msg);
//void print_end_state(moveit_msgs::DisplayTrajectory& the_plan);
void print_joint_state(moveit::planning_interface::MoveGroup& joint_group);
void get_end_state(moveit_msgs::DisplayTrajectory& the_plan, AdeptJoint* robot_joints, osu_ros_adept::robot_movement_command& movement_msg);


AdeptInterface *adept;

int main(int argc, char **argv){
 	ros::init(argc, argv, "programmatic_moveit");
  	ros::NodeHandle node_handle;
  	ros::AsyncSpinner spinner(1);
  	spinner.start();

	adept = new AdeptInterface();
	ros::Subscriber position_info = node_handle.subscribe("/convex_hull/adept_wrist_orientation", 1, get_hand_pose_request);

	ros::Duration wait_period(0.2);
	while(ros::ok()){
		adept->move_hand();
		wait_period.sleep();
	}

	return 0;
}

void get_hand_pose_request(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	adept->recv_new_hand_goal(msg);
}

AdeptInterface::AdeptInterface()
:group("adept_arm"), current_goal(new geometry_msgs::PoseStamped)
{
	display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/jc_planned_path", 1, true);
	action_request = node_handle.serviceClient<osu_ros_adept::robot_movement_command>("send_robot_movements");
	planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
	while(planning_scene_diff_publisher.getNumSubscribers() < 1)
	{
	  ros::WallDuration sleep_t(0.5);
	  sleep_t.sleep();    
	}
	add_table();

	cout << "Reference frame: " << group.getPlanningFrame() << endl;
	cout << "End effector: " << group.getEndEffectorLink() << endl;

	robot_joints = new AdeptJoint[6];
	has_new_goal = false;

	init_mk_movement_msg_template();

	group.setWorkspace(-1, -1, 0.4, 2, 2, 2);
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

void AdeptInterface::add_table()
{
	moveit_msgs::AttachedCollisionObject table = mk_table();
	moveit_msgs::PlanningScene planning_scene;
	planning_scene.world.collision_objects.push_back(table.object);
	planning_scene.is_diff = true;
	planning_scene_diff_publisher.publish(planning_scene); 
	
	cout << "Posted the table." << endl;
}

moveit_msgs::AttachedCollisionObject AdeptInterface::mk_table()
{
	double table_depth = 0.02;
	double table_length = 3;
	double table_width = 3;

	moveit_msgs::AttachedCollisionObject attached_object;
	attached_object.link_name = "Adept_Viper_s650_Link1";
	attached_object.object.header.frame_id = "Adept_Viper_s650_Link1";
	attached_object.object.id = "box";
	
	geometry_msgs::Pose pose;
	pose.orientation.w = 1.0;
	pose.orientation.x = pose.orientation.y = pose.orientation.z = 0;
	pose.position.z = -table_depth;
	pose.position.x = pose.position.y = 0;


	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;  
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = table_length;
	primitive.dimensions[1] = table_width;
	primitive.dimensions[2] = table_depth;  
	
	attached_object.object.primitives.push_back(primitive);
	attached_object.object.primitive_poses.push_back(pose);
	attached_object.object.operation = attached_object.object.ADD;

	return attached_object;
}

void AdeptInterface::recv_new_hand_goal(const geometry_msgs::PoseStamped::ConstPtr& goal)
{
	*current_goal = *goal;
	has_new_goal = true;
}

void AdeptInterface::move_hand()
{
	if (has_new_goal == false){
		return;
	} else {
		has_new_goal = false;
	}
	
	set_hand_goal();
	
	cout << "Beginning to plan." << endl;
	bool victory = group.plan(adept_plan);
	cout << "Finished planning." << endl;
	if (victory){
		cout << "Planning registered. New end effector positions:" << endl;
		print_joint_state(group);
		display_arm_trajectory();
		sleep(1);	
		//const robot_state::RobotState state = group.getJointValueTarget();
		//state.printStatePositions();
		get_end_state(display_trajectory, robot_joints, movement_msg);

		action_request.call(movement_msg);
		cout << "Bytes sent: " << movement_msg.response.bytes_sent << endl;

	} else {
		ROS_ERROR("No suitable motion plan found.");

	}
}

void AdeptInterface::set_hand_goal()
{
	//group.setPositionTarget(current_goal->pose.position.x, current_goal->pose.position.y, current_goal->pose.position.z);
	cout << "programmatic_moveit read position: x- " << current_goal->pose.position.x 
		<< " y- " << current_goal->pose.position.y << " z- " << current_goal->pose.position.z << endl;
	
	group.setPoseTarget(current_goal->pose);
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
	cout << "Num joints: " << num_joints << endl;
	for (int i = 0; i < num_joints; ++i){
		robot_joints[i].name = traj->joint_names[i]; 
		robot_joints[i].position = traj->points[final_pos_idx].positions[i];
		movement_msg.request.jts[i] = traj->points[final_pos_idx].positions[i];
		cout << "Joint " << robot_joints[i].name << " has position: " << robot_joints[i].position << endl;
	}
}
