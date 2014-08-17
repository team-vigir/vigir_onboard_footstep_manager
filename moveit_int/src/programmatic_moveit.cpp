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
	void set_hand_goal(const geometry_msgs::PoseStamped::ConstPtr& goal);
	void print_current_physical_joint_state();

private:
	void configure_movegroup();
	void init_mk_movement_msg_template();
	void add_table();
	moveit_msgs::AttachedCollisionObject mk_table();
	void display_arm_trajectory();
	void move_to_pregrasp_position();
	void plan_to_new_goal();
	void execute_trajectory();
	void set_movement_msg_with_next_trajectory(trajectory_msgs::JointTrajectoryPoint& current_waypoint);
	//void get_end_state(osu_ros_adept::robot_movement_command& movement_msg);

	ros::NodeHandle node_handle;
	moveit::planning_interface::MoveGroup group;
	moveit::planning_interface::MoveGroup::Plan adept_plan;
	AdeptJoint* robot_joints;
	osu_ros_adept::robot_movement_command movement_msg;
	geometry_msgs::PoseStamped current_goal;
	bool has_new_goal;

	ros::Publisher display_publisher;
	ros::Publisher planning_scene_diff_publisher;
	ros::ServiceClient action_request;
	moveit_msgs::DisplayTrajectory display_trajectory;

};

void recv_hand_pose_request(const geometry_msgs::PoseStamped::ConstPtr& msg);

AdeptInterface *adept;

int main(int argc, char **argv){
 	ros::init(argc, argv, "programmatic_moveit");
  	ros::NodeHandle node_handle;
  	ros::AsyncSpinner spinner(1);
  	
  	sleep(3);

  	spinner.start();

	adept = new AdeptInterface();
	ros::Subscriber position_info = node_handle.subscribe("/convex_hull/adept_wrist_orientation", 1, recv_hand_pose_request);

	ros::Duration wait_period(0.2);
	while(ros::ok()){
		adept->move_hand();
		wait_period.sleep();
	}

	return 0;
}

void recv_hand_pose_request(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	adept->set_hand_goal(msg);
}

AdeptInterface::AdeptInterface()
:group("adept_arm")
{
	configure_movegroup();

	display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/jc_planned_path", 1, true);
	action_request = node_handle.serviceClient<osu_ros_adept::robot_movement_command>("send_robot_movements");
	add_table();

	cout << "Reference frame: " << group.getPlanningFrame() << endl;
	cout << "End effector: " << group.getEndEffectorLink() << endl;

	robot_joints = new AdeptJoint[6];
	has_new_goal = false;

	init_mk_movement_msg_template();
}

void AdeptInterface::configure_movegroup()
{
	//group.setWorkspace(-1, -1, 0.4, 2, 2, 2); //Doesn't work
	double max_planning_time = 7;
	double orientation_rpy_radian_tolerance = 0.26;

	group.setPlanningTime(max_planning_time);
	group.setGoalOrientationTolerance(orientation_rpy_radian_tolerance);

	cout << "Limited planning time to " << max_planning_time << " seconds"
		<< " and added " << orientation_rpy_radian_tolerance * 57.295 << endl
		<< " degrees of rpy wiggle room to end effector pose." << endl << endl;
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
	planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
	cout << "Waiting for move_group to initialize properly before adding table..." << endl;
	ros::service::waitForService("/get_planning_scene", ros::Duration(10));

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

void AdeptInterface::set_hand_goal(const geometry_msgs::PoseStamped::ConstPtr& goal)
{
	cout << "programmatic_moveit read position: x- " << goal->pose.position.x 
		<< " y- " << goal->pose.position.y << " z- " << goal->pose.position.z << endl;
	
	current_goal = *goal;
	group.setPoseTarget(goal->pose);
	
	has_new_goal = true;
}

void AdeptInterface::move_hand()
{
	if (has_new_goal == false){
		return;
	
	} else {
		//move_to_pregrasp_position();
		plan_to_new_goal();
		has_new_goal = false;
	}
}

/*void AdeptInterface::move_to_pregrasp_position()
{
	Eigen::Quaterniond wrist_transform(current_goal.pose.orientation.w,
					current_goal.pose.orientation.x,
					current_goal.pose.orientation.y,
					current_goal.pose.orientation.z);
	Eigen::Vector3d palm_normal(0, 1, 0);
	palm_normal = wrist_transform._transformVector(palm_normal);
	palm_normal.normalize();

	Eigen::Vector3d offset = palm_normal * (-0.10);
	geometry_msgs::PoseStamped pregrasp_pose = current_goal;
	pregrasp_pose.pose.position.x += offset[0];
	pregrasp_pose.pose.position.y += offset[1];
	pregrasp_pose.pose.position.z += offset[2];

	group.setPoseTarget(pregrasp_pose.pose);
	plan_to_new_goal();

	group.setPoseTarget(current_goal.pose);
}*/

void AdeptInterface::plan_to_new_goal()
{
	if (group.plan(adept_plan)){
		cout << "Planning registered." << endl;
		display_arm_trajectory();
		execute_trajectory();

	} else {
		ROS_ERROR("No suitable motion plan found. We should add angular rotation.");

	}
}

void AdeptInterface::print_current_physical_joint_state()
{
	vector<string> joint_names = group.getJoints();
	vector<double> joint_positions = group.getCurrentJointValues();

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

/*void AdeptInterface::get_end_state(osu_ros_adept::robot_movement_command& movement_msg)
{
	long final_pos_idx = adept_plan.trajectory_.joint_trajectory.points.size() - 1;
	int num_joints = adept_plan.trajectory_.joint_trajectory.points[1].positions.size();

	if (final_pos_idx > 0){
		cout << "The motion plan generated has " << final_pos_idx << " waypoints!!!!! More than one!!! Could incorporate prgrasp position!" << endl;
	}

	trajectory_msgs::JointTrajectory* traj = &(adept_plan.trajectory_.joint_trajectory);
	cout << "Num joints: " << num_joints << endl;
	for (int i = 0; i < num_joints; ++i){
		robot_joints[i].name = traj->joint_names[i]; 
		robot_joints[i].position = traj->points[final_pos_idx].positions[i];
		movement_msg.request.jts[i] = traj->points[final_pos_idx].positions[i];
		cout << "Joint " << robot_joints[i].name << " has position: " << robot_joints[i].position << endl;
	}
}*/

void AdeptInterface::execute_trajectory()
{
	int num_waypoints = adept_plan.trajectory_.joint_trajectory.points.size();
	for (int i = 0; i < num_waypoints; i++){
		set_movement_msg_with_next_trajectory(adept_plan.trajectory_.joint_trajectory.points[i]);
		action_request.call(movement_msg);
		//cout << "Bytes sent: " << movement_msg.response.bytes_sent << endl;
		cout << "Sent out waypoint " << i << " to robot!" << endl;
	}
}

void AdeptInterface::set_movement_msg_with_next_trajectory(trajectory_msgs::JointTrajectoryPoint& current_waypoint)
{
	long num_joints = current_waypoint.positions.size();
	if (num_joints != 6){
		ROS_ERROR("num_joints is not 6 in programmatic_moveit set_movement_msg_with_next_trajectory!! Are we still using Adept?");
		exit(1);
	}

	for (int i = 0; i < num_joints; ++i){ 
		robot_joints[i].position = current_waypoint.positions[i];
		movement_msg.request.jts[i] = current_waypoint.positions[i];
		cout << "Joint " << i << " has position: " << robot_joints[i].position << endl;
	}
}
