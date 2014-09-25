/**************************************
* File: hullify_view.h
* Description: The following class contains the view
*	for this module, it will display any and every ros
*	message sent out of the program. It also includes
* 	convenience functions for message preparation.
* Author: Jackson Carter
* Date: 7/10/2014
***************************************/
#ifndef HULLIFY_VIEW_H
#define HULLIFY_VIEW_H

//ros includes
#include "ros/ros.h"
#include <ros/package.h>
#include "visualization_msgs/Marker.h"
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "osu_grasp_msgs/Mesh_and_bounds.h"
#include <shape_msgs/Plane.h>

#include "pcl/ros/conversions.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>

#include <Eigen/Dense>

#include <cstdlib>
#include <string>
#include <vector>
#include <map>
using std::string;
using std::vector;
using std::map;
using std::pair;
using std::cout;
using std::cin;
using std::endl;

void write_binary_stl(string full_abs_path, const pcl::PolygonMesh& mesh);

class Key_Compare {
	public:
		Key_Compare() {/*Nada*/};
		bool operator()(string key1, string key2);

};

class Hullify_View {
	public:
		Hullify_View();
		Hullify_View(string in_prefix, string* in_ff);
		~Hullify_View();

		//void template <class msg_type> add_topic(string name, msg_type type);
		template <class msg_type> void add_topic_no_queue(string name, msg_type type){
			name = prefix + name;
			ros::Publisher cur_topic = n.advertise<msg_type>(name, 1);
			pair<string, ros::Publisher> new_node(name, cur_topic);

			pair<map<string, ros::Publisher>::iterator, bool> ret_val = topics.insert(new_node);
			
			if (!ret_val.second){
				cout << "Could not add topic " << name << ". Topic already exists in Hullify_View." << endl;
			}

		};

		template <class msg_type> void publish(string name, msg_type msg){
			name = prefix + name;
			map<string, ros::Publisher>::iterator topic_node = topics.find(name);
			if (topic_node != topics.end()){
				topic_node->second.publish(msg);

			} else {
				cout << "Could not find topic " << name << " in Hullify_View::topics." << endl;

			}
		};

		void add_polygon_header(geometry_msgs::PolygonStamped& poly);
		visualization_msgs::Marker mk_pt_msg(Eigen::Vector3d pt);
		visualization_msgs::Marker mk_vector_msg(Eigen::Vector3d vector_rep);
		visualization_msgs::Marker mk_mesh_msg(string mesh_location);
		geometry_msgs::Polygon mk_square_plane_rep(pcl::PointXYZ center, pcl::PointXYZ max_pt, Eigen::Vector3d normal);
		geometry_msgs::Polygon mk_plane_rep_from_bounding_line(Eigen::Vector3d line_pt, Eigen::Vector3d slope, Eigen::Vector3d line_pt_to_bound);
		geometry_msgs::PoseStamped mk_pose_msg(Eigen::Quaterniond quat, Eigen::Vector3d pose_position);
		void add_mesh_topic(string base_name);
		string publish_mesh(string base_name, pcl::PolygonMesh::Ptr output_mesh);
		shape_msgs::Plane mk_shape_plane(pcl::ModelCoefficients plane);

	private:
		ros::NodeHandle n;
		string prefix;
		string* visualization_ref_frame;
		string mesh_folder_path;
		visualization_msgs::Marker template_marker;
		std::map <string, ros::Publisher, Key_Compare> topics;
		std::map <string, string, Key_Compare> meshes;

		void rm_prev_mesh(map<string, string>::iterator& mesh_info);
		string mk_mesh_path(string base_name);

		void common_constructor();
		void init_template_marker();
		void get_mesh_folder_path();
};

#endif
