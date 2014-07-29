#include "pose_transform.h"

/*bool vecs_are_equal(Eigen::Vector3d v1, Eigen::Vector3d v2)
{
	v1 /= v1.norm();
	v2 /= v2.norm();

	for (int i = 0; i < 3; ++i){	
		if (fabs(v1[i] - v2[i]) > FLOAT_TOLERANCE){
			return false;
		}
	}

	return true;
}*/

void print_quaternion(Eigen::Quaterniond quat)
{
	cout << "x: " << quat.x() << " y: " << quat.y() << " z: " << quat.z() << " w: " << quat.w() << endl;
}

//Preconditions: Valid, orthogonal axes are given.
//	such axes do not have a translation component, they are centered.
Eigen::Quaterniond get_axes_transformation(Axes initial_axes, Axes& goal_axes)
{
	Eigen::Quaterniond transform(1, 0, 0, 0);

	transform = align_axes_transform(initial_axes.z_axis, goal_axes.z_axis, initial_axes.x_axis) * transform;
	perform_axial_rotation(transform, initial_axes);
	
	Eigen::Vector3d negative_x_axis = -1 * initial_axes.x_axis;
	if (vecs_are_equal(initial_axes.x_axis, goal_axes.x_axis)){
		//No change
		cout << "no x rotation required." << endl;

	} else if (vecs_are_equal(negative_x_axis, initial_axes.x_axis)){
		transform = rotate_pi_about_axis(initial_axes.z_axis) * transform;
		cout << "180 degree rotation about z axis to align x axes" << endl;

	} else {
		double rotation_angle = find_angular_separation(initial_axes.x_axis, initial_axes.y_axis, goal_axes.x_axis);
		cout << "Rotation angle for x axes alignment: " << rotation_angle << endl;
		transform = get_rotational_quaternion(initial_axes.z_axis, rotation_angle) * transform;
		cout << "Rotational quaternion: " << endl; print_quaternion(get_rotational_quaternion(initial_axes.z_axis, rotation_angle));
	}
	
	cout << "Use the Y axis to verify the transformation in pose_transform.cpp" << endl;

	return transform;
}

Eigen::Quaterniond align_axes_transform(Eigen::Vector3d from_axis, Eigen::Vector3d to_axis, Eigen::Vector3d anti_parallel_rot_axis)
{
	Eigen::Quaterniond transform(1, 0, 0, 0), temp;
	Eigen::Vector3d negative_axis = -1 * to_axis;
	if (vecs_are_equal(from_axis, to_axis)){
		//No change;
		cout << "No change in z-axis direction required." << endl;
		cout << "initial: " << from_axis << endl << "goal: " << to_axis << endl;

	} else if(vecs_are_equal(from_axis, negative_axis)) {
		transform = rotate_pi_about_axis(anti_parallel_rot_axis);
		cout << "Rotation 180 degrees about the x-axis is required for the z-axis" << endl;
		
	} else {
		transform.setFromTwoVectors(from_axis, to_axis);
		transform.normalize();
		cout << "Abritrary transform required to align the z axes" << endl;
		
	}

	cout << "To align z-axis: "; print_quaternion(transform); cout << endl;
	return transform;	
}

Eigen::Quaterniond rotate_pi_about_axis(Eigen::Vector3d axis_of_rotation)
{
	double quat_w = 0;
	double quat_x = axis_of_rotation[0];
	double quat_y = axis_of_rotation[1];
	double quat_z = axis_of_rotation[2];

	Eigen::Quaterniond orientation_change(quat_w, quat_x, quat_y, quat_z);
	orientation_change.normalize();

	return orientation_change;
}

Eigen::Quaterniond get_rotational_quaternion(Eigen::Vector3d axis_of_rotation, double angle)
{
	double half_angle = angle / 2;
	double quat_w = cosf(half_angle);
	double quat_x = axis_of_rotation[0] * sinf(half_angle);
	double quat_y = axis_of_rotation[1] * sinf(half_angle);
	double quat_z = axis_of_rotation[2] * sinf(half_angle);

	Eigen::Quaterniond rotation_quat(quat_w, quat_x, quat_y, quat_z);
	rotation_quat.normalize();

	return rotation_quat;
}

void perform_axial_rotation(Eigen::Quaterniond rotation, Axes& axes)
{
	rotation.normalize();
	Eigen::Matrix<double,3,3> rotation_matrix = rotation.toRotationMatrix();
	axes.x_axis = rotation_matrix * axes.x_axis;
	axes.y_axis = rotation_matrix * axes.y_axis;
	axes.z_axis = rotation_matrix * axes.z_axis;
}

double find_angular_separation(Eigen::Vector3d ref, Eigen::Vector3d line_normal, Eigen::Vector3d vec)
{
	double projection_coefficient;
	double cos_theta = vec.dot(ref) / (vec.norm() * ref.norm());
	double rotation_angle = acos(cos_theta);

	if (isnan(rotation_angle)){
		if (cos_theta > 0)
			rotation_angle = 0;
		else
			rotation_angle = M_PI;
	}

	cout << "rotation angle preadjustment: " << rotation_angle << endl;

	projection_coefficient = calculate_parametric_coefficient_for_proj(ref, line_normal, vec);
	if (projection_coefficient < 0){
		rotation_angle = (2 * M_PI) - rotation_angle;

	}

	cout << "Rotation angle in find_angular_separation(): " << rotation_angle << endl;
	return rotation_angle;
}

double calculate_parametric_coefficient_for_proj(Eigen::Vector3d ref_slope, Eigen::Vector3d line_normal, Eigen::Vector3d vec_in_question)
{
	//double s, t;
/*	t = (line_normal[1] * vec_in_question[0]) - (line_normal[0] * vec_in_question[1]);
	t /= ((ref_slope[0] * line_normal[1]) - (ref_slope[1] * line_normal[0]));

	s = (ref_slope[0] * t) + vec_in_question[0];
	s /= line_normal[0];

	cout << "ref slope: " << ref_slope << endl << "line_normal: " << line_normal << endl;
	cout << "s: " << s << " t: " << t << " line_normal[0]: " << line_normal[0] << " t_denom: " << ((ref_slope[0] * line_normal[1]) - (ref_slope[1] * line_normal[0])) << endl;	
*/
	Eigen::Vector2d p(vec_in_question[0], vec_in_question[1]);
	Eigen::Vector2d n(line_normal[0], line_normal[1]);
	if (acos(p.dot(n) / (p.norm() * n.norm())) > (M_PI / 4)){
		cout << "Vector is opposite of normal." << endl;
		return -1;
	}

	cout << "Vector is on same side as normal." << endl;
	return 1;
}
