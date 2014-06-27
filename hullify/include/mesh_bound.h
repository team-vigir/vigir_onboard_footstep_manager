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
#include <pcl/filters/project_inliers.h>

#include <Eigen/Dense>

#include <cstdlib>
#include <string>
#include <vector>
using std::string;
using std::vector;
using std::cout;
using std::cin;
using std::endl;

inline Eigen::Vector3d init_vec(const tf::Vector3& in);
double pt_dist(pcl::PointXYZ pt1, pcl::PointXYZ pt2);

class MeshBound {
	public:
		MeshBound(string ff);
		MeshBound(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, string ff);

		geometry_msgs::Polygon* mk_plane_rep(pcl::PointXYZ center, pcl::PointXYZ max_pt, Eigen::Vector3d normal);
		void display_polygon(geometry_msgs::PolygonStamped polygon);
		void find_centroid();
		void find_plane();
		void set_input_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud);

		pcl::ModelCoefficients get_plane1();
		pcl::ModelCoefficients get_plane2();
		Eigen::Vector3d get_centroid();

		void publish_centroid();
		void publish_plane1();
		void publish_plane2();

	private:
		void init_template_marker();
		Eigen::Vector3d get_camera_normal();
		Eigen::Vector3d get_camera_position();

		void publish_normal(Eigen::Vector3d normal);
		void publish_proj_pts(pcl::PointCloud<pcl::PointXYZ>::Ptr proj_pts);
		pcl::PointXYZ find_farthest_pt(pcl::ModelCoefficients::Ptr plane);

		ros::NodeHandle n;
		string fixed_frame;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
		Eigen::Vector3d *centroid;
		pcl::ModelCoefficients::Ptr plane1;
		pcl::ModelCoefficients::Ptr plane2;

		visualization_msgs::Marker template_marker;
		ros::Publisher centroid_output;
		ros::Publisher plane1_output;
		ros::Publisher plane2_output;
		ros::Publisher normal_output;
		ros::Publisher proj_output;
};


#endif
