/***************************************************************************
* File: mesh_bound.h/.cpp
* Description: To decompose the making of an object from the
*   identification of its center and bounding planes, this object
*   was created. It requires the original pointcloud and can determine
*   all centroid and plane information.
* Notes: Centroid and plane data can be queried via getters; they are not
*   returned via methods.
***************************************************************************/
#ifndef MESH_BOUND_H
#define MESH_BOUND_H

//ros includes
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PolygonStamped.h>

#include <tf/transform_listener.h>

#include "pcl/ros/conversions.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/pca.h>

#include <Eigen/Dense>

#include "plane_reps_and_3dmath.h"
#include "hullify_view.h"

#include <cstdlib>
#include <string>
#include <vector>
using std::string;
using std::vector;
using std::cout;
using std::cin;
using std::endl;

//See find_max_radial_dist()
struct Pt_pos {
	long idx;	//Index into pointcloud for a point
	double angle;	//The point's radial position from reference
};

inline Eigen::Vector3d init_vec(const tf::Vector3& in);
int angle_compare(const void* a, const void* b);

class MeshBound {
	public:
		MeshBound();
		MeshBound(string ff, Hullify_View* in_view);
		MeshBound(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, string ff, Hullify_View* in_view);

		void display_polygon(geometry_msgs::PolygonStamped polygon);
		void construct_planes();
		void set_input_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud);

		pcl::ModelCoefficients get_plane1();
		pcl::ModelCoefficients get_plane2();
		Eigen::Vector3d get_centroid();
		Eigen::Vector3d get_camera_normal_vec();

		void publish_centroid();
		void publish_plane1();
		void publish_plane2();
		
	private:
		void constructor_common();
		void find_centroid();
		void get_camera_normal();
		void find_horiz_normal();
		Eigen::Vector3d get_camera_position();
		pcl::PointCloud<pcl::PointXYZ>::Ptr extract_middle_points(pcl::ModelCoefficients::Ptr horiz_plane);
		int* get_max_radial_dist(pcl::PointCloud<pcl::PointXYZ>::Ptr proj_pts);
		int* get_max_radial_dist_core(pcl::PointCloud<pcl::PointXYZ>::Ptr proj_pts, Eigen::Vector3d& ref_vec, Eigen::Vector3d& line_normal);
		Pt_pos* find_all_pt_angles(Eigen::Vector3d& ref_line_slope, Eigen::Vector3d& ref_line_orth, pcl::PointCloud<pcl::PointXYZ>::Ptr proj_pts, long& num_pts);
		bool pt_too_near_centroid(Pt_pos* pt_angles, long& pt_idx, Eigen::Vector3d& pt_to_centroid, long& num_pts);
		void print_pt_angles(Pt_pos* pt_angles, long num_pts);
		Eigen::Vector3d* get_ref_line_slope(pcl::PointCloud<pcl::PointXYZ>::Ptr proj_pts);
		bool pt_wraps_past_ref_line(const Eigen::Vector3d& slope, const Eigen::Vector3d& line_norm, pcl::PointXYZ& pt);
		bool is_valid_parametric_denom(int coord1, int coord2, Eigen::Vector3d slope, Eigen::Vector3d line_norm);
		void verify_pt_proj_using_third(int coord, Eigen::Vector3d pt1, Eigen::Vector3d pt2);
		bool determine_pt_hemisphere(double s);
		void calculate_parametric_coefficients_for_proj(int coord1, int coord2, Eigen::Vector3d slope, Eigen::Vector3d line_norm, Eigen::Vector3d pt, double& s, double& t);
		int* find_max_consecutive_angular_diff(Pt_pos* pt_angles, long num_pts);

		

		void publish_proj_pts(pcl::PointCloud<pcl::PointXYZ>::Ptr proj_pts);
		geometry_msgs::PolygonStamped mk_plane_msg(pcl::ModelCoefficients::Ptr plane);

		ros::NodeHandle n;
		string fixed_frame;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
		Eigen::Vector3d *centroid;
		Eigen::Vector3d camera_normal;	//This is a unit vector
		Eigen::Vector3d horiz_normal;
		pcl::ModelCoefficients::Ptr plane1;
		pcl::ModelCoefficients::Ptr plane2;

		Hullify_View* view;
};


#endif
