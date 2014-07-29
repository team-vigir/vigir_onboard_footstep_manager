/***************************************
* File: plane_reps_and_3dmath.h
* Description: This code handles creating planes in
*	3d space from vectors and points along with other 
*	necessary mathematical functionality (distance between
*	two points and projections)
****************************************/
#include "plane_reps_and_3dmath.h"

double difference(double var1, double var2)
{
	return fabs(var1 - var2);
}

bool vecs_are_equal(Eigen::Vector3d v1, Eigen::Vector3d v2)
{
	v1 /= v1.norm();
	v2 /= v2.norm();

	if (difference(v1[0], v2[0]) <= FLOAT_TOLERANCE
		&& difference(v1[1], v2[1]) <= FLOAT_TOLERANCE
		&& difference(v1[2], v2[2]) <= FLOAT_TOLERANCE){
		return true;
	}

	return false;
}

Eigen::Vector3d init_vec(const pcl::PointXYZ& in)
{
	Eigen::Vector3d out(in.x, in.y, in.z);
	return out;
}

pcl::PointXYZ init_pt(double x, double y, double z)
{
	pcl::PointXYZ pt;
	pt.x = x;
	pt.y = y;
	pt.z = z;

	return pt;
}

double pt_dist(pcl::PointXYZ pt1, pcl::PointXYZ pt2)
{
	double dx = pt1.x - pt2.x;
	double dy = pt1.y - pt2.y;
	double dz = pt1.z - pt2.z;

	return sqrt((dx * dx) + (dy * dy) + (dz * dz));
}

pcl::PointXYZ find_farthest_pt(pcl::PointXYZ location, pcl::PointCloud<pcl::PointXYZ>::Ptr other_points)
{
	long num_pts = other_points->points.size();
	double max_dist = 0;
	double this_dist;
	pcl::PointXYZ* most_distal_point = NULL;
	for (long i = 0; i < num_pts; ++i){
		this_dist = pt_dist(location, other_points->points[i]);
		
		if (this_dist > max_dist){
			most_distal_point = &(other_points->points[i]);
			max_dist = this_dist;
		}
	}
	
	if (most_distal_point == NULL){
		cout << "Could not find a most distal point from input pointcloud in find_farthest_pt(). Returning (0,0,0)";
		return pcl::PointXYZ (0, 0, 0);
	}

	return *most_distal_point;
}

//[a,b,c] is the normal vector of the plane
pcl::ModelCoefficients::Ptr init_plane(double a, double b, double c, double d)
{
	pcl::ModelCoefficients::Ptr new_plane (new pcl::ModelCoefficients);
	new_plane->values.resize(4);
	new_plane->values[0] = a;
	new_plane->values[1] = b;
	new_plane->values[2] = c;
	new_plane->values[3] = d;

	return new_plane;
}

bool planes_are_eq(pcl::ModelCoefficients::Ptr in_plane1, pcl::ModelCoefficients::Ptr in_plane2)
{
	pcl::ModelCoefficients plane1; 
	plane1.values = in_plane1->values;
	pcl::ModelCoefficients plane2;
	plane2.values = in_plane2->values;

	normalize_plane(&plane1);
	normalize_plane(&plane2);

	cout << "Plane1: " << plane1.values[0] << "  " << plane1.values[1] << "  " << plane1.values[2] << "  " << plane1.values[3] << endl; 
	cout << "Plane2: " << plane2.values[0] << "  " << plane2.values[1] << "  " << plane2.values[2] << "  " << plane2.values[3] << endl; 

	return ((difference(plane1.values[0], plane2.values[0]) < FLOAT_TOLERANCE)
		&& (difference(plane1.values[1], plane2.values[1]) < FLOAT_TOLERANCE)
		&& (difference(plane1.values[2], plane2.values[2]) < FLOAT_TOLERANCE)
		&& (difference(plane1.values[3], plane2.values[3]) < FLOAT_TOLERANCE)) || 
	((difference(plane1.values[0], -plane2.values[0]) < FLOAT_TOLERANCE)
		&& (difference(plane1.values[1], -plane2.values[1]) < FLOAT_TOLERANCE)
		&& (difference(plane1.values[2], -plane2.values[2]) < FLOAT_TOLERANCE)
		&& (difference(plane1.values[3], -plane2.values[3]) < FLOAT_TOLERANCE));
}

void normalize_plane(pcl::ModelCoefficients* plane)
{
	Eigen::Vector3d normal(plane->values[0], plane->values[1], plane->values[2]);
	
	double normalizing_divisor = normal.norm();
	for (unsigned char i = 0; i < 4; ++i){
		plane->values[i] = plane->values[i] / normalizing_divisor;
	}

}

void normalize_plane(pcl::ModelCoefficients::Ptr plane)
{
	Eigen::Vector3d normal(plane->values[0], plane->values[1], plane->values[2]);
	
	double normalizing_divisor = normal.norm();
	for (unsigned char i = 0; i < 4; ++i){
		plane->values[i] = plane->values[i] / normalizing_divisor;
	}

}

//This function simply converts
//	a point and a normal vector into a plane
//	in pcl
//Eq: ax + by + cz + d = 0
//PCL needs Hessian Normal Form for its plane coefficients...
pcl::ModelCoefficients::Ptr mk_plane(Eigen::Vector3d pt, Eigen::Vector3d normal)
{
	//Construct missing plane coefficient
	double d = -1 * (pt.dot(normal));

	//Create the plane
	double hess_denom = normal.norm();
	normal = normal/hess_denom;
	
	pcl::ModelCoefficients::Ptr plane = init_plane(normal[0], normal[1], normal[2], d / hess_denom);

	cout << "New plane coefficients (mk_plane): " << "\n\ta: " << plane->values[0]
		<< "\tb: " << plane->values[1] << "\tc: " << plane->values[2]
		<< "\td: " << plane->values[3] << endl;

	return plane;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr project_pts_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, pcl::ModelCoefficients::Ptr plane)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr proj_pts (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setInputCloud(in_cloud);
	proj.setModelCoefficients(plane);
	proj.filter(*proj_pts);

	return proj_pts;
}

//Preconditions: the plane is in hessian normal form.
//		The cloud pts contains at least two points
//Returns two point references; first is min, second is max.
void find_max_min_dist_from_plane(pcl::ModelCoefficients::Ptr hess_norm_plane, pcl::PointCloud<pcl::PointXYZ>::Ptr pts, int* min_max)
{
	long num_pts = pts->points.size();
	Pt_Dist* dist_arr = new Pt_Dist[num_pts];
	
	for (long i = 0; i < num_pts; ++i){
		dist_arr[i].idx = i;
		dist_arr[i].dist = pt_to_plane_dist(hess_norm_plane, pts->points[i]);
	}

	qsort(dist_arr, num_pts, sizeof(Pt_Dist), dist_compare);
	print_max_min_distances(dist_arr, num_pts);

	min_max[0] = dist_arr[0].idx;
	min_max[1] = dist_arr[num_pts - 1].idx;
	delete [] dist_arr;
}

int dist_compare(const void* a, const void* b)
{
	Pt_Dist* pt1 = (Pt_Dist*) a;
	Pt_Dist* pt2 = (Pt_Dist*) b;

	if (pt1->dist > pt2->dist){
		return 1;

	} else if (pt1->dist < pt2->dist){
		return -1;
	}

	return 0;
}

void print_max_min_distances(Pt_Dist* dist_arr, long num_pts)
{
	cout << "Printing ordered distances from pointcloud to horizontal plane:" << endl;
	for (int i = 0; i < num_pts; ++i){
		cout << "\tPt_dist " << i << " cloud_idx: " << dist_arr[i].idx 
				<< " distance: " << dist_arr[i].dist << endl;
	}

	cout << endl;
}

//Preconditions: The plane must be in hessian normal form
//	 (unit normal vector).
//Return Value: <0 is in opposite direction of normal. >0 is in
//	line with normal.
double pt_to_plane_dist(pcl::ModelCoefficients::Ptr hess_norm_plane, const pcl::PointXYZ& pt)
{
	double dist = hess_norm_plane->values[0]*pt.x + hess_norm_plane->values[1]*pt.y + hess_norm_plane->values[2]*pt.z;
	dist += hess_norm_plane->values[3];
	//cout << "Pt to plane dist: " << dist << endl;
	return dist;

}

//Does not require a hessian normal form plane.
//Separation distance is in meters and is positive.
//Returns: "lower" (opposite normal) plane comes first
void mk_sandwich_planes(pcl::ModelCoefficients::Ptr orig_plane, double dist_orig_to_new, pcl::ModelCoefficients::Ptr* out_planes)
{
	pcl::ModelCoefficients::Ptr sandwich_plane_top = init_plane(0,0,0,0);
	pcl::ModelCoefficients::Ptr sandwich_plane_bot = init_plane(0,0,0,0);

	Eigen::Vector3d pt_on_plane = vec_to_pt_on_plane(orig_plane);
	//cout << "mk_sandwich_planes pt: " << endl << pt_on_plane << endl;
	Eigen::Vector3d unit_norm(orig_plane->values[0], orig_plane->values[1], orig_plane->values[2]);
	unit_norm /= unit_norm.norm();
	//cout << "mk_sandwich_planes unit vec:" << endl << unit_norm << endl;

	sandwich_plane_top = mk_plane((pt_on_plane + dist_orig_to_new*unit_norm), unit_norm);
	sandwich_plane_bot = mk_plane((pt_on_plane - dist_orig_to_new*unit_norm), unit_norm);

	out_planes[0] = sandwich_plane_bot;
	out_planes[1] = sandwich_plane_top;
}

//Preconditions: The input plane is a valid plane.
//Plane Equation: ax + by + cz + d = 0
Eigen::Vector3d vec_to_pt_on_plane(pcl::ModelCoefficients::Ptr plane)
{
	//d = 0 implies the origin is in the plane always
	double d = plane->values[3];
	if (d == 0){
		return Eigen::Vector3d(0,0,0);
	}

	Eigen::Vector3d pt_on_plane(0,0,0);
	double cur_coefficient;
	for (unsigned char i = 0; i < 3; ++i){
		cur_coefficient = plane->values[i];
		if (cur_coefficient != 0){
			pt_on_plane[i] = -(d / cur_coefficient);
			break;
		}
	}

	return pt_on_plane;
}

//Parameters: ordered_planes: The first plane is beneath the second
pcl::PointCloud<pcl::PointXYZ>::Ptr find_pts_between_parallel_planes(pcl::ModelCoefficients::Ptr* ordered_planes, pcl::PointCloud<pcl::PointXYZ>::Ptr pts)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pts_in_between (new pcl::PointCloud<pcl::PointXYZ>);
	normalize_plane(ordered_planes[0]);
	normalize_plane(ordered_planes[1]);

	long num_pts = pts->points.size();
	for (long i = 0; i < num_pts; ++i){
		if (pt_above_plane(ordered_planes[0], pts->points[i])
			&& pt_below_plane(ordered_planes[1], pts->points[i])){
			pts_in_between->points.push_back(pts->points[i]);
			//cout << "Adding point to return cloud in find_pts_between_parallel_planes." << endl;
		}
	}

	return pts_in_between;
}

bool pt_above_plane(pcl::ModelCoefficients::Ptr plane, pcl::PointXYZ& pt)
{
	
	double dist = pt_to_plane_dist(plane, pt);
	//cout << "Plane a:" << plane->values[0] << " b:" << plane->values[1] << " c:" << plane->values[2] << " d:" << plane->values[3] << endl;
	//cout << "Point(above) x:" << pt.x << " y:" << pt.y << " z:" << pt.z << " Dist: " << dist << endl << endl; 
	return (dist + FLOAT_TOLERANCE >= 0);
}

bool pt_below_plane(pcl::ModelCoefficients::Ptr plane, pcl::PointXYZ& pt)
{
	double dist = pt_to_plane_dist(plane, pt);
	//cout << "Plane a:" << plane->values[0] << " b:" << plane->values[1] << " c:" << plane->values[2] << " d:" << plane->values[3] << endl;
	//cout << "Point(below) x:" << pt.x << " y:" << pt.y << " z:" << pt.z << " Dist: " << dist << endl << endl; 
	return (dist - FLOAT_TOLERANCE <= 0);
}