#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

//#include "mesh_bound.h"
#include "plane_reps_and_3dmath.h"

#include <tf/tf.h>

#include <eigen3/Eigen/Dense>

#include <iostream>
#include <cmath>

using std::cout;
using std::cin;
using std::endl;

#define FLOAT_TOLERANCE 0.001


struct Axes {
	Eigen::Vector3d x_axis;
	Eigen::Vector3d y_axis;
	Eigen::Vector3d z_axis;
};


//bool vecs_are_equal(Eigen::Vector3d v1, Eigen::Vector3d v2);
Eigen::Quaterniond get_axes_transformation(Axes inital_axes, Axes& goal_axes);
Eigen::Quaterniond align_axes_transform(Eigen::Vector3d from_axis, Eigen::Vector3d to_axis, Eigen::Vector3d anti_parallel_rot_axis);
Eigen::Quaterniond rotate_pi_about_axis(Eigen::Vector3d axis_of_rotation);
Eigen::Quaterniond get_rotational_quaternion(Eigen::Vector3d axis_of_rotation, double angle);
double find_angular_separation(Eigen::Vector3d ref, Eigen::Vector3d line_normal, Eigen::Vector3d vec);
double calculate_parametric_coefficient_for_proj(Eigen::Vector3d ref_slope, Eigen::Vector3d line_normal, Eigen::Vector3d vec_in_question);
void perform_axial_rotation(Eigen::Quaterniond rotation, Axes& axes);
void print_quaternion(Eigen::Quaterniond quat);
