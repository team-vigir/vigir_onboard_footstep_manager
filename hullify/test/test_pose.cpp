#include "pose_transform.h"
#include "gtest/gtest.h"

#define TESTING_POSES

bool operator==(Eigen::Quaterniond q1, Eigen::Quaterniond q2)
{
	q1.normalize();
	q2.normalize();

	if (fabs(q1.w() - q2.w()) < FLOAT_TOLERANCE
		&& fabs(q1.x() - q2.x()) < FLOAT_TOLERANCE
		&& fabs(q1.y() - q2.y()) < FLOAT_TOLERANCE
		&& fabs(q1.z() - q2.z()) < FLOAT_TOLERANCE){
			return true;
	}

	return false;
}

Axes mk_simple_axes()
{
	Axes simple_axes;
	simple_axes.x_axis = Eigen::Vector3d(1, 0, 0);
	simple_axes.y_axis = Eigen::Vector3d(0, 1, 0);
	simple_axes.z_axis = Eigen::Vector3d(0, 0, 1);

	return simple_axes;	
}

TEST(rotate_pi_about_axis, all_axes){
	Eigen::Vector3d x_axis(1, 0, 0);
	Eigen::Vector3d y_axis(0, 1, 0);
	Eigen::Vector3d z_axis(0, 0, 1);
	Eigen::Vector3d scaled_axis (2, 0, 0);
	Eigen::Vector3d negative_axis (0, -1, 0);
	Eigen::Vector3d arbitrary_axis (0.25, 51, -3);

	EXPECT_TRUE(Eigen::Quaterniond(0, 1, 0, 0) == rotate_pi_about_axis(x_axis));
	EXPECT_TRUE(Eigen::Quaterniond(0, 0, 1, 0) == rotate_pi_about_axis(y_axis));
	EXPECT_TRUE(Eigen::Quaterniond(0, 0, 0, 1) == rotate_pi_about_axis(z_axis));
	EXPECT_TRUE(Eigen::Quaterniond(0, 1, 0, 0) == rotate_pi_about_axis(scaled_axis));
	EXPECT_TRUE(Eigen::Quaterniond(0, 0, -1, 0) == rotate_pi_about_axis(negative_axis));
	
	Eigen::Quaterniond quat6(0, 0.25, 51, -3); quat6.normalize();
	EXPECT_TRUE(quat6 == rotate_pi_about_axis(arbitrary_axis));
}

TEST(get_rotational_quaternion, quarter_turns){
	Eigen::Vector3d z_axis(0, 0, 1);

	double root2_over2 = sqrt(2) / 2;
	Eigen::Quaterniond first_quarter(root2_over2, 0, 0, root2_over2);
	first_quarter.normalize();
	Eigen::Quaterniond third_quarter(-root2_over2, 0, 0, root2_over2);
	third_quarter.normalize();
	EXPECT_TRUE(first_quarter == get_rotational_quaternion(z_axis, M_PI/2));
	EXPECT_TRUE(Eigen::Quaterniond(0, 0, 0, 1) == get_rotational_quaternion(z_axis, M_PI));
	EXPECT_TRUE(third_quarter == get_rotational_quaternion(z_axis, (3*M_PI) / 2));
	EXPECT_TRUE(Eigen::Quaterniond(-1, 0, 0, 0) == get_rotational_quaternion(z_axis, 2*M_PI));
}

TEST(get_rotational_quaternion, odd_axes){
	Eigen::Vector3d weird_axis(-.25, 1, 15);
	double rotation_angle = M_PI / 6;

	Eigen::Quaterniond result(0.965925, -0.06470476, 0.258819, 3.882285);
	result.normalize();
	cout << "expected result: "; print_quaternion(result);
	cout << "actual quaternion: "; print_quaternion(get_rotational_quaternion(weird_axis, rotation_angle));
	ASSERT_TRUE(result == get_rotational_quaternion(weird_axis, rotation_angle));
}

TEST(perform_axial_rotation, simple_axes){
	Axes simple_axes = mk_simple_axes();
	Axes copy_axes = simple_axes;

	Eigen::Quaterniond simple_rotation(0, 1, 0, 0);
	perform_axial_rotation(simple_rotation, simple_axes);
	EXPECT_TRUE(vecs_are_equal(Eigen::Vector3d(1, 0, 0), simple_axes.x_axis));
	EXPECT_TRUE(vecs_are_equal(Eigen::Vector3d(0, -1, 0), simple_axes.y_axis));
	EXPECT_TRUE(vecs_are_equal(Eigen::Vector3d(0, 0, -1), simple_axes.z_axis));

	Eigen::Quaterniond not_so_simple_rotation = get_rotational_quaternion(Eigen::Vector3d(1, 1, -2), M_PI / 3);
	perform_axial_rotation(not_so_simple_rotation, copy_axes);
	print_quaternion(not_so_simple_rotation);
	EXPECT_TRUE(vecs_are_equal(Eigen::Vector3d(-.11111, -.547577, -.829344), copy_axes.x_axis));
	EXPECT_TRUE(vecs_are_equal(Eigen::Vector3d(.992022, -.11111, -.059544), copy_axes.y_axis));
	EXPECT_TRUE(vecs_are_equal(Eigen::Vector3d(-.059544, -0.829344, .55555), copy_axes.z_axis));

	cout << "x_axis: " << copy_axes.x_axis << endl;
	cout << "y_axis: " << copy_axes.y_axis << endl;
	cout << "z_axis: " << copy_axes.z_axis << endl;
}



/***** Integration Tests ***** <-- extract to new file*/
TEST(full_test_run, trivial_transform){
	Axes simple_axes = mk_simple_axes();
	Eigen::Quaterniond trivial_quat(1, 0, 0, 0);
	
	EXPECT_TRUE(trivial_quat == get_axes_transformation(simple_axes, simple_axes));
}

TEST(full_test_run, z_axis_flipped){
	Axes simple_axes = mk_simple_axes();
	Axes rot_about_x_axis = mk_simple_axes();
	rot_about_x_axis.z_axis = Eigen::Vector3d(0, 0, -1);
	rot_about_x_axis.y_axis = Eigen::Vector3d(0, -1, 0);
	Eigen::Quaterniond x_rotate_pi(0, 1, 0, 0);

	EXPECT_TRUE(x_rotate_pi == get_axes_transformation(simple_axes, rot_about_x_axis));
}

TEST(full_test_run, simple_x_rotation_CCW){
	Axes simple_axes = mk_simple_axes();
	Axes tilted_axes = simple_axes;
	tilted_axes.z_axis = Eigen::Vector3d(0, -1, 1);
	tilted_axes.y_axis = Eigen::Vector3d(0, 1, 1);
	Eigen::Quaterniond simple_ccw_rotation(.92388, .382683, 0, 0);

	EXPECT_TRUE(simple_ccw_rotation == get_axes_transformation(simple_axes, tilted_axes));
}

TEST(x_rotation, small_x_rotation){
	Axes  simple_axes = mk_simple_axes();
	Axes rotated_xy_axis = simple_axes;
	rotated_xy_axis.x_axis = Eigen::Vector3d(1, 1, 0);
	rotated_xy_axis.y_axis = Eigen::Vector3d(-1, 1, 0);
	Eigen::Quaterniond simple_x_rotation(0.923879, 0, 0, .382683);

	EXPECT_TRUE(simple_x_rotation == get_axes_transformation(simple_axes, rotated_xy_axis));
	EXPECT_TRUE(fabs(0.785398 - find_angular_separation(simple_axes.x_axis, simple_axes.y_axis, rotated_xy_axis.x_axis)) < 0.01);

}

TEST(x_rotation, greater_than_pi_rotation){

}

TEST(arbitrary_rotation, arb_rot){
	Axes simple_axes = mk_simple_axes();
	Axes goal_axes;
	goal_axes.x_axis = Eigen::Vector3d(1, -1, 0);
	goal_axes.y_axis = Eigen::Vector3d(0, 0, -1);
	goal_axes.z_axis = Eigen::Vector3d(1, 1, 0);
	Eigen::Quaterniond res(0.5, -0.853552, -0.1464459, 0.707106);

	EXPECT_TRUE(Eigen::Quaterniond(0.707107, -0.707107, 0.707107, 0) == align_axes_transform(simple_axes.z_axis, goal_axes.z_axis, simple_axes.x_axis));
	EXPECT_TRUE(res == get_axes_transformation(simple_axes, goal_axes));
}

int main(int argc, char** argv){
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
