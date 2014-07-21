#include "plane_reps_and_3dmath.h"
#include "gtest/gtest.h"

pcl::ModelCoefficients::Ptr mk_xy_plane()
{
	pcl::ModelCoefficients::Ptr xy_plane (new pcl::ModelCoefficients);
	xy_plane->values.resize(4);
	xy_plane->values[0] = 0;
	xy_plane->values[1] = 0;
	xy_plane->values[2] = 1;
	xy_plane->values[4] = 0;

	return xy_plane;
}

TEST(pt_to_plane_dist, on_plane){
	pcl::ModelCoefficients::Ptr xy_plane = mk_xy_plane();

	EXPECT_EQ(0, pt_to_plane_dist(xy_plane, init_pt(0,0,0)));
	EXPECT_EQ(0, pt_to_plane_dist(xy_plane, init_pt(5,5,0)));
	EXPECT_EQ(0, pt_to_plane_dist(xy_plane, init_pt(3, -4, 0)));
}

TEST(pt_to_plane_dist, above_plane){
	pcl::ModelCoefficients::Ptr xy_plane = mk_xy_plane();

	EXPECT_EQ(1, pt_to_plane_dist(xy_plane, init_pt(0,0,1)));
	EXPECT_EQ(1.5, pt_to_plane_dist(xy_plane, init_pt(3,4,1.5)));
	EXPECT_GT(FLOAT_TOLERANCE, difference(55.36, pt_to_plane_dist(xy_plane, init_pt(0,6,55.36))));
}

TEST(pt_to_plane_dist,  below_plane){
	Eigen::Vector4d normal(-1, 0, 1, 0);
	normal /= normal.norm();
	pcl::ModelCoefficients::Ptr plane = init_plane(normal[0], normal[1], normal[2], normal[3]);

	EXPECT_GT(FLOAT_TOLERANCE, difference(-1.0606, pt_to_plane_dist(plane, init_pt(2, 34.5, 0.5))));
}

TEST(find_max_min_pts, minimal_cloud){
	pcl::PointCloud<pcl::PointXYZ>::Ptr small_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	small_cloud->points.push_back(init_pt(0,0,0));
	small_cloud->points.push_back(init_pt(1,1,1));

	pcl::ModelCoefficients::Ptr xy_plane = mk_xy_plane();
	int min_max[2];
	find_max_min_dist_from_plane(xy_plane, small_cloud, min_max);

	EXPECT_EQ(0, min_max[0]);
	EXPECT_EQ(1, min_max[1]);

	small_cloud->points.push_back(init_pt(4,7,-1.5));
	small_cloud->points.push_back(init_pt(1,2,0.5));

	find_max_min_dist_from_plane(xy_plane, small_cloud, min_max);
	EXPECT_EQ(2, min_max[0]);
	EXPECT_EQ(1, min_max[1]);

}

TEST(find_max_min_pts, equal_pts){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_on_plane(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_on_plane->points.push_back(init_pt(1,2,0));
	cloud_on_plane->points.push_back(init_pt(3.5,6,0));
	cloud_on_plane->points.push_back(init_pt(1.00000001,302,0));

	pcl::ModelCoefficients::Ptr xy_plane = mk_xy_plane();
	int min_max[2];
	find_max_min_dist_from_plane(xy_plane, cloud_on_plane, min_max);

	EXPECT_EQ(0, min_max[0]);
	EXPECT_EQ(2, min_max[1]);
}

TEST(vec_to_pt_on_plane, d_equal_zero){
	//This plane must contain the origin
	pcl::ModelCoefficients::Ptr plane1 = init_plane(1, 2, 3, 0);
	pcl::ModelCoefficients::Ptr plane2 = init_plane(0, 1, 1, 0);
	pcl::ModelCoefficients::Ptr plane3 = init_plane(2.225, 0, 1.5, 0);

	Eigen::Vector3d pt = vec_to_pt_on_plane(plane1);
	EXPECT_EQ(Eigen::Vector3d(0,0,0), pt);

	pt = vec_to_pt_on_plane(plane2);
	EXPECT_EQ(Eigen::Vector3d(0,0,0), pt);

	pt = vec_to_pt_on_plane(plane3);
	EXPECT_EQ(Eigen::Vector3d(0,0,0), pt);
}

TEST(vec_to_pt_on_plane, arb_planes){
	pcl::ModelCoefficients::Ptr plane1 = init_plane(1, 2, 3, -1);
	pcl::ModelCoefficients::Ptr plane2 = init_plane(0, 1, 1, -5);
	pcl::ModelCoefficients::Ptr plane3 = init_plane(0, 0, 5, -1);
	pcl::ModelCoefficients::Ptr plane4 = init_plane(0, 0, 0, -1);

	Eigen::Vector3d pt = vec_to_pt_on_plane(plane1);
	EXPECT_EQ(Eigen::Vector3d(1,0,0), pt);

	pt = vec_to_pt_on_plane(plane2);
	EXPECT_EQ(Eigen::Vector3d(0,5,0), pt);

	pt = vec_to_pt_on_plane(plane3);
	EXPECT_EQ(Eigen::Vector3d(0,0,0.2), pt);

	pt = vec_to_pt_on_plane(plane4);
	EXPECT_EQ(Eigen::Vector3d(0,0,0), pt);
}

//Try distances in centimenters on abritrary planes
TEST(mk_sandwich_planes, normal_dist){
	pcl::ModelCoefficients::Ptr rand_plane = init_plane(-5, 0, .3, 1);
	pcl::ModelCoefficients::Ptr planes[2];
	
	mk_sandwich_planes(rand_plane, 0.05, planes);
	EXPECT_TRUE(planes_are_eq(init_plane(-5, 0, 0.3, 1.25044), planes[0]));
	EXPECT_TRUE(planes_are_eq(init_plane(-5, 0, 0.3, 0.74955), planes[1]));
}

TEST(mk_sandwich_planes, xy_yz_xz){
	pcl::ModelCoefficients::Ptr plane1 = init_plane(0, 0, 1, -1);
	pcl::ModelCoefficients::Ptr plane2 = init_plane(0, 1, 0, -1);
	pcl::ModelCoefficients::Ptr plane3 = init_plane(1, 0, 0, -1);

	pcl::ModelCoefficients::Ptr planes[2];
	mk_sandwich_planes(plane1, 2, planes);
	EXPECT_TRUE(planes_are_eq(init_plane(0, 0, 1, 1), planes[0]));
	EXPECT_TRUE(planes_are_eq(init_plane(0, 0, 1, -3), planes[1]));

	mk_sandwich_planes(plane2, 2, planes);
	EXPECT_TRUE(planes_are_eq(init_plane(0, 1, 0, 1), planes[0]));
	EXPECT_TRUE(planes_are_eq(init_plane(0, 1, 0, -3), planes[1]));

	mk_sandwich_planes(plane3, 2, planes);
	EXPECT_TRUE(planes_are_eq(init_plane(1, 0, 0, 1), planes[0]));
	EXPECT_TRUE(planes_are_eq(init_plane(1, 0, 0, -3), planes[1]));
}

TEST(mk_sandwich_planes, zero_dist){
	pcl::ModelCoefficients::Ptr plane1 = init_plane(-7.2, 5, 1, 1);
	pcl::ModelCoefficients::Ptr planes[2];
	mk_sandwich_planes(plane1, 0, planes);

	EXPECT_TRUE(planes_are_eq(plane1, planes[0]));
	EXPECT_TRUE(planes_are_eq(plane1, planes[1]));
}

TEST(planes_are_eq, simple_test){
	pcl::ModelCoefficients::Ptr plane1 = init_plane(1, 1, 1, 1);
	pcl::ModelCoefficients::Ptr plane2 = init_plane(2, 2, 2, 2);

	EXPECT_TRUE(planes_are_eq(plane1, plane1));
	EXPECT_TRUE(planes_are_eq(plane1, plane2));

	EXPECT_EQ(2, plane2->values[3]) << "The plane equatily checker changed plane coefficients.";
	EXPECT_EQ(2, plane2->values[0]);
}

TEST(find_pts_between_parallel_planes, empty_cloud){
	pcl::PointCloud<pcl::PointXYZ>::Ptr empty(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ModelCoefficients::Ptr xy_plane = mk_xy_plane();
	pcl::ModelCoefficients::Ptr z_eq_1 = init_plane(0, 0, 1, -1);

	pcl::ModelCoefficients::Ptr planes[2] = {xy_plane, z_eq_1};
	pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud = find_pts_between_parallel_planes(planes, empty);

	EXPECT_EQ(0, out_cloud->points.size());
}

TEST(find_pts_between_parallel_planes, no_pts_on_plane){
	pcl::PointCloud<pcl::PointXYZ>::Ptr pts(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ModelCoefficients::Ptr xy_plane = mk_xy_plane();
	pcl::ModelCoefficients::Ptr z_eq_1 = init_plane(0, 0, 1, -1);
	pcl::ModelCoefficients::Ptr planes[2] = {xy_plane, z_eq_1};

	pts->points.push_back(init_pt(0.5, 0.5, 0.25));
	pts->points.push_back(init_pt(1, 2, 0.5));
	pts->points.push_back(init_pt(20, .17, -0.5));
	pts->points.push_back(init_pt(.556, -2, 2));
	pts->points.push_back(init_pt(3, 2, 0.5));

	pcl::PointCloud<pcl::PointXYZ>::Ptr res_cloud = find_pts_between_parallel_planes(planes, pts);
	ASSERT_EQ(3, res_cloud->points.size());
	EXPECT_EQ(Eigen::Vector3d(0.5, 0.5, 0.25), init_vec(res_cloud->points[0]));
	EXPECT_EQ(Eigen::Vector3d(1, 2, 0.5), init_vec(res_cloud->points[1]));
	EXPECT_TRUE(vecs_are_equal(Eigen::Vector3d(3, 2, 0.5), init_vec(res_cloud->points[2])));
}

TEST(find_pts_between_parallel_planes, pts_on_planes){
	pcl::PointCloud<pcl::PointXYZ>::Ptr pts(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ModelCoefficients::Ptr rand_plane = init_plane(1, 1, 1, -1);
	pcl::ModelCoefficients::Ptr planes[2];
	mk_sandwich_planes(rand_plane, 0.5, planes);

	pts->points.push_back(init_pt(0.9, 0, 0));
	pts->points.push_back(init_pt(1, 12, 0.5));
	pts->points.push_back(init_pt(1.28867513, 0.28867513, 0.28867513));
	pts->points.push_back(init_pt(0.71132486, -0.28867513, -0.28867513));
	pts->points.push_back(init_pt(.556, -2, -2));

	pcl::PointCloud<pcl::PointXYZ>::Ptr res_cloud = find_pts_between_parallel_planes(planes, pts);
	ASSERT_EQ(3, res_cloud->points.size());
	EXPECT_TRUE(vecs_are_equal(Eigen::Vector3d(0.9, 0, 0), init_vec(res_cloud->points[0])));
	EXPECT_TRUE(vecs_are_equal(Eigen::Vector3d(1.28867513, 0.28867513, 0.28867513), init_vec(res_cloud->points[1])));
	EXPECT_TRUE(vecs_are_equal(Eigen::Vector3d(0.71132486, -0.28867513, -0.28867513), init_vec(res_cloud->points[2])));
}


int main(int argc, char** argv){
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}