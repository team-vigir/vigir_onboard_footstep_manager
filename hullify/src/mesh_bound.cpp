#include "mesh_bound.h"

inline Eigen::Vector3d init_vec(const tf::Vector3& in)
{
	Eigen::Vector3d out(in.m_floats[0], in.m_floats[1], in.m_floats[2]);
	return out;
}

MeshBound::MeshBound()
{
	cout << "Default constructor for MeshBound called. Error!!" << endl;
	exit(1);

}

//Default constructor, just nullifies things.
MeshBound::MeshBound(string ff, Hullify_View* in_view)
:cloud (new pcl::PointCloud<pcl::PointXYZ>),
 view (in_view)
{
	fixed_frame = ff;
	constructor_common();
	//cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr();	//NULL
}

MeshBound::MeshBound(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, string ff, Hullify_View* in_view)
:cloud (new pcl::PointCloud<pcl::PointXYZ>),
 view (in_view)
{
	fixed_frame = ff;
	constructor_common();
	*cloud = *in_cloud;
}

void MeshBound::constructor_common()
{
	centroid = NULL;
	plane1 = pcl::ModelCoefficients::Ptr();
	plane2 = pcl::ModelCoefficients::Ptr();
	//perception_link = "/hokuyo_link";
	perception_link = "/palm";

	visualization_msgs::Marker marker_type;
	geometry_msgs::PolygonStamped poly_type;
	sensor_msgs::PointCloud2 cloud_type;
	view->add_topic_no_queue("centroid", marker_type);
	view->add_topic_no_queue("normal", marker_type);
	view->add_topic_no_queue("horiz_normal", marker_type);
	view->add_topic_no_queue("plane1", poly_type);
	view->add_topic_no_queue("plane2", poly_type);
	view->add_topic_no_queue("horiz_projected", cloud_type);
	view->add_topic_no_queue("camera_pos", marker_type);
}

//Description: Changes the input cloud, invalidates the centroid
//	along with present planes. Does not necessarily delete old cloud.
void MeshBound::set_input_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud)
{
	*cloud = *in_cloud;

	delete centroid;
	centroid = NULL;
	camera_normal = Eigen::Vector3d(0, 0, 0);
	horiz_normal = Eigen::Vector3d(0, 0, 0);
	plane1 = pcl::ModelCoefficients::Ptr();
	plane2 = pcl::ModelCoefficients::Ptr();

	find_centroid();
}

//construct_planes is the organizer of plane creation
//	it will determine the normal of a plane 
//	to project all input points onto, will find
//	the largest angular difference between two
//	consecutive points and construct the planes
void MeshBound::construct_planes()
{
	if (centroid == NULL || cloud == pcl::PointCloud<pcl::PointXYZ>::Ptr()){
		cout << "Either the centroid or the input cloud is not present. Please set them." << endl;
		return;
	}
	
	//Find the normal of the first projective plane
	visualization_msgs::Marker horiz_msg;
	find_horiz_normal();
	horiz_msg = view->mk_vector_msg(horiz_normal);
	view->publish("horiz_normal", horiz_msg);

	//Project points
	pcl::ModelCoefficients::Ptr horiz_plane = mk_plane(*centroid, horiz_normal);
	pcl::PointCloud<pcl::PointXYZ>::Ptr middle_points = extract_middle_points(horiz_plane);
	pcl::PointCloud<pcl::PointXYZ>::Ptr proj_pts = project_pts_plane(middle_points, horiz_plane);
	publish_proj_pts(proj_pts);

	//Find largest d_theta
	int* pt_idxs = get_max_radial_dist(proj_pts);
	if (pt_idxs == NULL){
		cout << "Generating NULL Planes for viewing." << endl;
		plane1 = mk_plane(*centroid, camera_normal);
		plane2 = mk_plane(*centroid, camera_normal);
		return;

	}
	
	//Construct planes
	Eigen::Vector3d v1 = init_vec(proj_pts->points[pt_idxs[0]]) - *centroid;
	Eigen::Vector3d v2 = init_vec(proj_pts->points[pt_idxs[1]]) - *centroid;
	Eigen::Vector3d n1 = horiz_normal.cross(v1);
	Eigen::Vector3d n2 = horiz_normal.cross(v2);

	plane1 = mk_plane(*centroid, n1);
	plane2 = mk_plane(*centroid, n2);

	cout << "Normal for plane 1: " << endl << n1 << endl;
	cout << "Normal for plane 2: " << endl << n2 << endl;
	cout << "v2: " << endl << v2 << endl;
	cout << "second point: " << endl << init_vec(proj_pts->points[pt_idxs[1]]) << endl;
	delete [] pt_idxs;
}

void MeshBound::find_horiz_normal()
{
	get_camera_normal();
	Eigen::Vector3d k(0, 0, 1);

	double angle_from_k = acos(k.dot(camera_normal));
	cout << "Angle from k: " << angle_from_k << endl;

	if (angle_from_k < 0.01 || fabs(angle_from_k - M_PI) < 0.01 || isnan(angle_from_k)){
		//Looking straight down or up; Any horizontal angle will work
		cout << "The camera is directly above or beneath centroid; defining arbitrary horiz_normal." << endl;
		horiz_normal = Eigen::Vector3d(1,0,0);
	
	} else {
		Eigen::Vector3d temp = camera_normal.cross(k);
		horiz_normal = (-1 * camera_normal).cross(temp);
	}

	cout << "horiz_normal in find_horiz_normal(): " << endl << horiz_normal << endl;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MeshBound::extract_middle_points(pcl::ModelCoefficients::Ptr horiz_plane)
{
	int min_max[2];
	pcl::ModelCoefficients::Ptr planes[2];

	find_max_min_dist_from_plane(horiz_plane, cloud, min_max);
	double middle_band_width = 0.15 * difference((cloud->points)[min_max[1]].z, (cloud->points)[min_max[0]].z);
	mk_sandwich_planes(horiz_plane, middle_band_width / 2, planes);

	pcl::PointCloud<pcl::PointXYZ>::Ptr middle_pts = find_pts_between_parallel_planes(planes, cloud);
	middle_pts->header = cloud->header;
	return middle_pts;
}

//Description: The program goes to each point in the 
//	input cloud and determines the angle between
//	the point and the centroid with respect to a reference
//	vector. The program then orders the angles and 
//	finds the largest d_theta between two consecutive ones.
//Preconditions: The points must have already been projected
//	onto a plane.
//Returns: The indices of the two points or NULL if an error
//	occurred.
//Math: Angle between vectors = cos-1(u . v / |u||v|)
int* MeshBound::get_max_radial_dist(pcl::PointCloud<pcl::PointXYZ>::Ptr proj_pts)
{
	//Create reference vector and pt_pos array
	Eigen::Vector3d *ref;
	int* out_array;

	if (proj_pts->points.size() < 3)
		return NULL;

    if ((ref = get_ref_line_slope(proj_pts)) == NULL){
		out_array = NULL;

	} else {
		Eigen::Vector3d line_normal = ref->cross(horiz_normal);
		cout << "Line normal: " << endl << line_normal << endl;
		cout << "Ref: " << endl << *ref << endl;

		out_array = get_max_radial_dist_core(proj_pts, *ref, line_normal);
		delete ref;
	}

	return out_array;

}

//Returns: Null, or a vector from the centroid
//	to a suitable point on the plane to define a reference.
Eigen::Vector3d* MeshBound::get_ref_line_slope(pcl::PointCloud<pcl::PointXYZ>::Ptr proj_pts)
{
	Eigen::Vector3d* ref = new Eigen::Vector3d;
	long num_pts = proj_pts->points.size();
	bool found_slope = false;
	for (long i = 0; i < num_pts; ++i){
		*ref = init_vec(proj_pts->points[i]) - *centroid;
		
		if (ref->norm() > .01){
			found_slope = true;
			break;
		}
	}

	if (!found_slope){
		ROS_ERROR("The figure in question is not large enough to generate an accurate plane boundary.");
		ROS_ERROR("Please select another pointcloud.");
		return NULL; 
	}

	*ref = *ref / ref->norm(); // |u| = 1;

	return ref;
}

//Description: Does the heavy lifting behind its
//	calling function: it finds all point angles, orders them
//	and returns the largest difference between two consecutive points.
int* MeshBound::get_max_radial_dist_core(pcl::PointCloud<pcl::PointXYZ>::Ptr proj_pts, Eigen::Vector3d& ref_vec, Eigen::Vector3d& line_normal)
{
	long num_pts;
	Pt_pos* pt_angles = find_all_pt_angles(ref_vec, line_normal, proj_pts, num_pts);

	qsort(pt_angles, num_pts, sizeof(Pt_pos), angle_compare);
	print_pt_angles(pt_angles, num_pts);

	int* cur_max_pts = find_max_consecutive_angular_diff(pt_angles, num_pts);

	cout << "Largest Angular difference: between cloud_idx0:" << cur_max_pts[0] 
		<< " and cloud_idx1: " << cur_max_pts[1] << endl;
	
	delete [] pt_angles;
	return cur_max_pts;
}

Pt_pos* MeshBound::find_all_pt_angles(Eigen::Vector3d& ref_line_slope, Eigen::Vector3d& ref_line_orth, pcl::PointCloud<pcl::PointXYZ>::Ptr proj_pts, long& num_pts)
{
	num_pts = proj_pts->points.size();
	Pt_pos* pt_angles = new Pt_pos[num_pts];
	Eigen::Vector3d cur_vec;
	for (long i = 0; i < num_pts; ++i){
		cur_vec = init_vec(proj_pts->points[i]) - *centroid;
		
		if (pt_too_near_centroid(pt_angles, i, cur_vec, num_pts)){
			continue;
		}

		pt_angles[i].idx = i;
		pt_angles[i].angle = acos(ref_line_slope.dot(cur_vec) / cur_vec.norm());
		if (isnan(pt_angles[i].angle)){
			cout << "acos returned nan angle in MeshBound::find_all_pt_angles(). Setting to 0." << endl;
			if (ref_line_slope.dot(cur_vec) / cur_vec.norm() > 0)
				pt_angles[i].angle = 0;
			else 
				pt_angles[i].angle = M_PI;
		}

		//Reflect point if it is past PI rad
		if (pt_wraps_past_ref_line(ref_line_slope, ref_line_orth, proj_pts->points[i])){
			pt_angles[i].angle = 2 * M_PI - pt_angles[i].angle;
		}

		cout << "\tCloud_idx " << pt_angles[i].idx << "(" << proj_pts->points[i].x << " , " << proj_pts->points[i].y << " , " << proj_pts->points[i].z << ") has angle " 
			<< pt_angles[i].angle << endl;
	}

	return pt_angles;
}

//This function will skip insertion of a 
//	point if it is too close to the centroid
bool MeshBound::pt_too_near_centroid(Pt_pos* pt_angles, long& pt_idx, Eigen::Vector3d& pt_to_centroid, long& num_pts)
{
	if (pt_to_centroid.norm() < .01){
		cout << "Skipping angle, too close to centroid. idx = " << pt_idx << endl;
		num_pts--;
		pt_idx--;

		return true;
	}

	return false;
}

//Returns true if the point is beyond the
//	reference normal (on the other side of a
//	cross-sectional line)
//Line: Centroid + t(camera_normal)
bool MeshBound::pt_wraps_past_ref_line(const Eigen::Vector3d& slope, const Eigen::Vector3d& line_norm, pcl::PointXYZ& pt)
{
	int X = 0, Y = 1, Z = 2;
	int verification_coord = 0;
	double s;	//The coefficient of the line_normal
	double t;	//The scalar for the coefficient of camera_norm (slope)
	Eigen::Vector3d pt_in_question = init_vec(pt);

	//See Doc folder in hullify package for mathematical explanation
	if (is_valid_parametric_denom(X, Y, slope, line_norm)){
		calculate_parametric_coefficients_for_proj(X, Y, slope, line_norm, pt_in_question, s, t);
		verification_coord = Z;

	} else if (is_valid_parametric_denom(Y, X, slope, line_norm)){
		calculate_parametric_coefficients_for_proj(Y, X, slope, line_norm, pt_in_question, s, t);
		verification_coord = Z;

	} else if (is_valid_parametric_denom(X, Z, slope, line_norm)){
		calculate_parametric_coefficients_for_proj(X, Z, slope, line_norm, pt_in_question, s, t);
		verification_coord = Y;

	} else if (is_valid_parametric_denom(Z, X, slope, line_norm)){
		calculate_parametric_coefficients_for_proj(Z, X, slope, line_norm, pt_in_question, s, t);
		verification_coord = Y;

	} else if (is_valid_parametric_denom(Z, Y, slope, line_norm)){
		calculate_parametric_coefficients_for_proj(Z, Y, slope, line_norm, pt_in_question, s, t);
		verification_coord = X;

	} else if (is_valid_parametric_denom(Y, Z, slope, line_norm)){
		calculate_parametric_coefficients_for_proj(Y, Z, slope, line_norm, pt_in_question, s, t);
		verification_coord = X;
	}

	verify_pt_proj_using_third(verification_coord, slope*t + (*centroid), line_norm*s + init_vec(pt));

	cout.precision(9);
	cout << "\t\ts: " << s << " t: " << t << endl;

	return determine_pt_hemisphere(s);
}

bool MeshBound::is_valid_parametric_denom(int coord1, int coord2, Eigen::Vector3d slope, Eigen::Vector3d line_norm)
{
	return (((slope[coord1] * line_norm[coord2]) - (slope[coord2] * line_norm[coord1])) != 0
			&& line_norm[coord1] != 0);
}

void MeshBound::calculate_parametric_coefficients_for_proj(int coord1, int coord2, Eigen::Vector3d slope, Eigen::Vector3d line_norm, Eigen::Vector3d pt, double& s, double& t)
{
	t = line_norm[coord2] * (pt[coord1] - (*centroid)[coord1]) - (line_norm[coord1] * (pt[coord2] - (*centroid)[coord2]));
	t /= ((slope[coord1] * line_norm[coord2]) - (slope[coord2] * line_norm[coord1]));

	s = (slope[coord1] * t) + (*centroid)[coord1] - pt[coord1];
	s /= line_norm[coord1];
}

void MeshBound::verify_pt_proj_using_third(int coord, Eigen::Vector3d pt1, Eigen::Vector3d pt2)
{
	if (fabs(pt1[coord] - pt2[coord]) > 0.01){
		ROS_ERROR("pt_wraps_past_ref_line() verification indicates error in projection.");
	}
}

//Description: s is the coefficient on the vector needed to 
//	project the point above onto the reference line. t is the 
//	coefficient on the on the reference line's slope to reach the
//	projected point.
//	s > 0 implies upper hemisphere. s < 0 implies lower hemisphere
//	if s == 0, the point does not need to be reflected.
bool MeshBound::determine_pt_hemisphere(double s)
{
	if (s >= 0){
		return false;	
	}

	return true;	//Default: it is a big angle
}

void MeshBound::print_pt_angles(Pt_pos* pt_angles, long num_pts)
{
	cout << endl << "Aftering Sorting:" << endl;
	for (int i = 0; i < num_pts; ++i){
		cout << "Pt_angles " << i << " cloud_idx: " << pt_angles[i].idx 
			<< " ang: " << pt_angles[i].angle << endl;
	}
}

//Find and return largest difference
/*prev_idx is an index into the array of pt_pos structs
 * 	The actual index of the point, however, is in the struct itself*/
int* MeshBound::find_max_consecutive_angular_diff(Pt_pos* pt_angles, long num_pts)
{
	int* cur_max_pts = new int[2];

	int idx;
	int prev_idx = 0;
	double prev_max = 0;
	double cur_angle;
	for (long i = 1; i < (num_pts + 1); ++i){
		idx = i % num_pts;
		cur_angle = fabs(pt_angles[idx].angle - pt_angles[prev_idx].angle);
		
		if (cur_angle > M_PI){
			cur_angle = 2 * M_PI - cur_angle;
		}

		if(cur_angle > prev_max){
			prev_max = cur_angle;
			cur_max_pts[0] = pt_angles[idx].idx;
			cur_max_pts[1] = pt_angles[prev_idx].idx;
			cout << endl << "\tNew largest angle: pt_idx1 - " << idx << " pt_idx2 - " 
				<< prev_idx << " d_theta = "
				<<(pt_angles[idx].angle - pt_angles[prev_idx].angle) << endl;
		}

		prev_idx = idx;
	}

	return cur_max_pts;
}

int angle_compare(const void* a, const void* b)
{
	struct Pt_pos* pt1 = (struct Pt_pos*) a;
	struct Pt_pos* pt2 = (struct Pt_pos*) b;

	if (pt1->angle > pt2->angle){
		return 1;	//pt1 goes after

	} else if (pt1->angle < pt2->angle){
		return -1;	//pt1 goes before
	}

	return 0;	//they're equal
}



void MeshBound::get_camera_normal()
{
	//Find the camera's current position
	Eigen::Vector3d camera_pos = get_camera_position();

	//Draw a vector between it and the centroid
	camera_normal = Eigen::Vector3d(*centroid - camera_pos);

	//Make it a unit vector
	camera_normal = camera_normal / camera_normal.norm();

	cout << "Normal vector from camera to centroid: " << endl << camera_normal << endl;
}

Eigen::Vector3d MeshBound::get_camera_position()
{
	tf::TransformListener listener;

	tf::StampedTransform transform;
	while (1){
		try {
			listener.lookupTransform(fixed_frame.c_str(), perception_link.c_str(),
						ros::Time(0), transform);
		} catch (tf::TransformException ex){
			ROS_ERROR("%s", ex.what());
			sleep(1);
			continue;
		}

		break;
	}
	
	tf::Vector3 pos = transform.getOrigin();

	//For testing!
	//pos.m_floats[0] = -1.5; pos.m_floats[1] = 0.5; pos.m_floats[2] = 2;
	//pos.m_floats[0] = .5; pos.m_floats[1] = .5; pos.m_floats[2] = 2;	
	//pos.m_floats[0] = .5; pos.m_floats[1] = -1; pos.m_floats[2] = 0;
	//pos.m_floats[0] = 0; pos.m_floats[1] = -1; pos.m_floats[2] = 0;
	//pos.m_floats[0] = 0; pos.m_floats[1] = -1; pos.m_floats[2] = 0.5;
	pos.m_floats[0] = 2; pos.m_floats[1] = 0; pos.m_floats[2] = 0;
	visualization_msgs::Marker camera_pos_msg = view->mk_pt_msg(init_vec(pos));
	view->publish("camera_pos", camera_pos_msg);

	return init_vec(pos);
}

//Function: find_centroid()
//Description: Given a raw pointcloud, this function estimates
//	the object's center of mass by averaging the x, y, and z
//	values of all points.
//Assumptions: Uniform density and a representative, equally distributed
//	 point cloud
//Parameters: cloud - the original queried pointcloud for the object.
void MeshBound::find_centroid()
{
	long num_pts = cloud->points.size();
	double sum_x = 0;
	double sum_y = 0;
    double sum_z = 0;
	for (long i = 0; i < num_pts; ++i){
		sum_x += cloud->points[i].x;
		sum_y += cloud->points[i].y;
		sum_z += cloud->points[i].z;
	}

	centroid = new Eigen::Vector3d;
	*centroid = Eigen::Vector3d((sum_x / num_pts), (sum_y / num_pts), (sum_z / num_pts));
}

/****** Getters
************************/
pcl::ModelCoefficients MeshBound::get_plane1()
{
	pcl::ModelCoefficients out_plane;
	if (plane1 != pcl::ModelCoefficients::Ptr()){
		out_plane = *plane1;

	} else {
		cout << "No valid plane found, returning empty plane." << endl;
		out_plane.values.resize(4);
		out_plane.values[0] = out_plane.values[1] = out_plane.values[2] = out_plane.values[3] = 0;

	}

	return out_plane;

}

//Returns a copy of plane 1 if plane2 is non-existant.
pcl::ModelCoefficients MeshBound::get_plane2()
{
	pcl::ModelCoefficients out_plane;
	if (plane2 != pcl::ModelCoefficients::Ptr()){
		out_plane = *plane2;

	} else if (plane1 != pcl::ModelCoefficients::Ptr()){
		out_plane = *plane1;

	} else {
		cout << "No valid plane found, returning empty plane." << endl;
		out_plane.values.resize(4);
		out_plane.values[0] = out_plane.values[1] = out_plane.values[2] = out_plane.values[3] = 0;

	}

	return out_plane;

}

//Returns a copy of the centroid for local use.
Eigen::Vector3d MeshBound::get_centroid()
{
	if (centroid != NULL){
		return *centroid;
	}

	return Eigen::Vector3d(0, 0, 0);
}

Eigen::Vector3d MeshBound::get_camera_normal_vec()
{
	if (centroid != NULL){
		return camera_normal;
	}

	return Eigen::Vector3d(0, 0, 0);
}


/****** Visualization
**********************************/
void MeshBound::publish_centroid()
{
	visualization_msgs::Marker center_marker = view->mk_pt_msg(*centroid);
	view->publish("centroid", center_marker);
}

void MeshBound::publish_plane1()
{
	geometry_msgs::PolygonStamped plane = mk_plane_msg(plane1);
	view->publish("plane1", plane);
}
void MeshBound::publish_plane2()
{
	geometry_msgs::PolygonStamped plane = mk_plane_msg(plane2);
	view->publish("plane2", plane);
}

 geometry_msgs::PolygonStamped MeshBound::mk_plane_msg(pcl::ModelCoefficients::Ptr plane)
{
	pcl::PointXYZ center ((*centroid)[0], (*centroid)[1], (*centroid)[2]);
	pcl::PointXYZ max_pt = find_farthest_pt(center, cloud);
	Eigen::Vector3d temp = init_vec(max_pt) - *centroid;

	Eigen::Vector3d normal(plane->values[0], plane->values[1], plane->values[2]);
	Eigen::Vector3d max_vec = normal.cross(horiz_normal);
	max_vec *= (temp.norm() / max_vec.norm());

	geometry_msgs::PolygonStamped out_poly;
	view->add_polygon_header(out_poly);
	geometry_msgs::Polygon poly = view->mk_plane_rep_from_bounding_line(*centroid, horiz_normal, max_vec);
	out_poly.polygon = poly;
	
	return out_poly;
}

void MeshBound::publish_proj_pts(pcl::PointCloud<pcl::PointXYZ>::Ptr proj_pts)
{
	cout << "Publishing projected points:" << endl;
	sensor_msgs::PointCloud2 temp2;
	pcl::toROSMsg(*proj_pts, temp2);
	view->publish("horiz_projected", temp2);
	//cout << "One projected point: " << proj_pts->points[0].x << "  " 
		//<< proj_pts->points[0].y << "  " << proj_pts->points[0].z << endl;

}
