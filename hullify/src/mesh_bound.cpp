#include "mesh_bound.h"

inline Eigen::Vector3d init_vec(const tf::Vector3& in)
{
	Eigen::Vector3d out(in.m_floats[0], in.m_floats[1], in.m_floats[2]);
	return out;
}

//Default constructor, just nullifies things.
MeshBound::MeshBound(string ff)
:cloud (new pcl::PointCloud<pcl::PointXYZ>)
{
	centroid = NULL;
	//cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr();	//NULL
	plane1 = pcl::ModelCoefficients::Ptr();
	plane2 = pcl::ModelCoefficients::Ptr();
	fixed_frame = ff;

	centroid_output = n.advertise<visualization_msgs::Marker>("/convex_hull/centroid", 5);
	normal_output = n.advertise<visualization_msgs::Marker>("/convex_hull/normal", 5);
	plane1_output = n.advertise<geometry_msgs::PolygonStamped>("/convex_hull/plane1", 5);
	plane2_output = n.advertise<geometry_msgs::PolygonStamped>("/convex_hull/plane2", 5);
	proj_output = n.advertise<sensor_msgs::PointCloud2>("/convex_hull/proj", 5);

	init_template_marker();
}

MeshBound::MeshBound(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, string ff)
:cloud (new pcl::PointCloud<pcl::PointXYZ>)
{
	centroid = NULL;
	*cloud = *in_cloud;
	plane1 = pcl::ModelCoefficients::Ptr();
	plane2 = pcl::ModelCoefficients::Ptr();
	fixed_frame = ff;

	centroid_output = n.advertise<visualization_msgs::Marker>("/convex_hull/centroid", 5);
	normal_output = n.advertise<visualization_msgs::Marker>("/convex_hull/normal", 5);
	plane1_output = n.advertise<geometry_msgs::PolygonStamped>("/convex_hull/plane1", 5);
	plane2_output = n.advertise<geometry_msgs::PolygonStamped>("/convex_hull/plane2", 5);
	proj_output = n.advertise<sensor_msgs::PointCloud2>("/convex_hull/projected_pts", 5);

	init_template_marker();
}

//Preconditions: f_name (the filename for the mesh), must be
//  set prior to this call.
void MeshBound::init_template_marker()
{
	template_marker.header.frame_id = fixed_frame;
	template_marker.header.stamp = ros::Time::now();
	template_marker.header.seq = 1;
	template_marker.action = visualization_msgs::Marker::ADD;
	template_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
	template_marker.ns = "plane_ns";
	template_marker.id = 0;

	template_marker.pose.position.x = 0;    //Where it goes
	template_marker.pose.position.y = 0;
	template_marker.pose.position.z = 0;

	template_marker.pose.orientation.x = 0.0;   //What's its pose.
	template_marker.pose.orientation.y = 0.0;
	template_marker.pose.orientation.z = 0.0;
	template_marker.pose.orientation.w = 1.0;

	template_marker.scale.x = 1;    //Resizing required?
	template_marker.scale.y = 1;
	template_marker.scale.z = 1;

	template_marker.color.a = 1.0;
	template_marker.color.r = 0.5;
	template_marker.color.g = 1.0;
	template_marker.color.b = 0.0;

	template_marker.lifetime = ros::Duration(0);
	template_marker.mesh_use_embedded_materials = false;
}

//Description: Changes the input cloud and invalidates the centroid
//	and present planes. Does not necessarily delete old cloud.
void MeshBound::set_input_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud)
{
	*cloud = *in_cloud;

	delete centroid;
	centroid = NULL;
	plane1 = pcl::ModelCoefficients::Ptr();
	plane2 = pcl::ModelCoefficients::Ptr();
}

//For simplicity, the function simply defines the
//	normal vector from the centroid to the position
//	of the camera. We then use the plane equation
//Eq: ax + by + cz + d = 0
//PCL needs Hessian Normal Form for its plane coefficients...
void MeshBound::find_plane()
{
	if (centroid == NULL || cloud == pcl::PointCloud<pcl::PointXYZ>::Ptr()){
		cout << "Either the centroid or the input cloud is not present. Please set them." << endl;
		return;
	}

	//Get the normal
	Eigen::Vector3d normal = get_camera_normal();
	publish_normal(normal);

	//Construct plane coefficients
	double d = -1 * (centroid->dot(normal));

	//Create the plane
	plane1 = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients());
	double hess_denom = sqrt(pow(normal[0], 2) + pow(normal[1], 2) + pow(normal[2], 2)); //sqrt(a^2 + b^2 + c^2)
	plane1->values.resize(4);
	plane1->values[0] = normal[0] / hess_denom;
	plane1->values[1] = normal[1] / hess_denom;
	plane1->values[2] = normal[2] / hess_denom;
	plane1->values[3] =  d / hess_denom;

	cout << "Plane coefficients: " << "\n\ta: " << plane1->values[0]
		<< "\tb: " << plane1->values[1] << "\tc: " << plane1->values[2]
		<< "\td: " << plane1->values[3] << endl;
}

Eigen::Vector3d MeshBound::get_camera_normal()
{
	//Find the camera's current position
	Eigen::Vector3d camera_pos = get_camera_position();

	//Draw a vector between it and the centroid
	Eigen::Vector3d normal(*centroid - camera_pos);
	cout << "Normal vector: " << normal << endl;

	return normal;
}

Eigen::Vector3d MeshBound::get_camera_position()
{
	tf::TransformListener listener;

	tf::StampedTransform transform;
	while (1){
		try {
			listener.lookupTransform(fixed_frame.c_str(), "/hokuyo_link",
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
	
	cout << "The camera has position:" << endl << "\tX: " << pos.m_floats[0]
		<< endl << "\tY: " << pos.m_floats[1] << endl << "\tZ: " << pos.m_floats[2] << endl;
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
	if (centroid != NULL)
		return;	//The centroid has already been created for this cloud

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

double pt_dist(pcl::PointXYZ pt1, pcl::PointXYZ pt2)
{
	double dx = pt1.x - pt2.x;
	double dy = pt1.y - pt2.y;
	double dz = pt1.z - pt2.z;

	return sqrt((dx * dx) + (dy * dy) + (dz * dz));
}

geometry_msgs::Polygon* MeshBound::mk_plane_rep(pcl::PointXYZ center, pcl::PointXYZ max_pt, Eigen::Vector3d normal)
{
	geometry_msgs::Polygon* out_poly = new geometry_msgs::Polygon;

	//Make maximum and scale for viewing
	Eigen::Vector3d max((max_pt.x - center.x), (max_pt.y - center.y), (max_pt.z - center.z));
	max = max * 1.5;
	//cout << "Max Projected Vector:(scaled) " << max;

	//Make a square where orth points to another vertex
	Eigen::Vector3d orth = max.cross(normal);
	orth = (orth / orth.norm()) * max.norm();

	geometry_msgs::Point32 temp;
	temp.x = center.x + max[0]; temp.y = center.y + max[1]; temp.z = center.z + max[2];
	out_poly->points.push_back(temp);
	temp.x = center.x + orth[0]; temp.y = center.y + orth[1]; temp.z = center.z + orth[2];
	out_poly->points.push_back(temp);
	temp.x = center.x - max[0]; temp.y = center.y - max[1]; temp.z = center.z - max[2];
	out_poly->points.push_back(temp);
	temp.x = center.x - orth[0]; temp.y = center.y - orth[1]; temp.z = center.z - orth[2];
	out_poly->points.push_back(temp);

	return out_poly;
}

//Description: Project the current cloud onto the 
//	created plane and determine the point that is farthest
//	from the centroid.
//Preconditions: The given plane exists and is properly initialized.
pcl::PointXYZ MeshBound::find_farthest_pt(pcl::ModelCoefficients::Ptr plane)
{
	//Project all points
	pcl::PointCloud<pcl::PointXYZ>::Ptr proj_pts (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setInputCloud(cloud);
	proj.setModelCoefficients(plane);
	proj.filter(*proj_pts);

	publish_proj_pts(proj_pts);

	//Find farthest point from centroid
	//cout << "Centroid in publish_plane(): " << endl << centroid << endl;
	long num_pts = proj_pts->points.size();
	double max_dist = 0;
	double cur_dist;
	pcl::PointXYZ* cur_max = NULL;
	pcl::PointXYZ center((*centroid)[0], (*centroid)[1], (*centroid)[2]);
	for (long i = 0; i < num_pts; ++i){
		cur_dist = pt_dist(center, proj_pts->points[i]);
		if (cur_dist > max_dist){
			cur_max = &(proj_pts->points[i]);
		}
	}

	return *cur_max;
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
	return *centroid;
}


/****** Visualization
**********************************/

//This function is for debugging purposes,
//	it makes a disposable marker for showing
//	the centroid. It is inefficient.
void MeshBound::publish_centroid()
{
	//Make a new marker
	visualization_msgs::Marker center_marker = template_marker;
	center_marker.type = visualization_msgs::Marker::SPHERE;
	center_marker.pose.position.x = (*centroid)[0];
	center_marker.pose.position.y = (*centroid)[1];
	center_marker.pose.position.z = (*centroid)[2];
	center_marker.color.b = 1;
	center_marker.color.g = 0;
	center_marker.id = 1;
	
	//Set the size of the marker.
	center_marker.scale.x = .1;
	center_marker.scale.y = .1;
	center_marker.scale.z = .1;

	centroid_output.publish(center_marker);
}

void MeshBound::publish_normal(Eigen::Vector3d normal)
{
	visualization_msgs::Marker normal_marker = template_marker;
	normal_marker.type = visualization_msgs::Marker::ARROW;
	normal_marker.id = 2;
	normal_marker.color.r = 1.0;
	normal_marker.color.g = .25;
	
	normal_marker.scale.x = .1;
	normal_marker.scale.y = .15;
	normal_marker.scale.z = .1;

	geometry_msgs::Point temp;
	temp.x = temp.y = temp.z = 0;
	normal_marker.points.push_back(temp);
	temp.x = normal[0]; temp.y = normal[1]; temp.z = normal[2];
	normal_marker.points.push_back(temp);

	normal_output.publish(normal_marker);
}

void MeshBound::publish_plane1()
{
	pcl::PointXYZ cur_max = find_farthest_pt(plane1);
	pcl::PointXYZ center((*centroid)[0], (*centroid)[1], (*centroid)[2]);

	geometry_msgs::Polygon* poly = mk_plane_rep(center, cur_max, Eigen::Vector3d(plane1->values[0], plane1->values[1], plane1->values[2]));
	geometry_msgs::PolygonStamped out_poly;
	out_poly.header.stamp = ros::Time::now();
	out_poly.header.frame_id = fixed_frame;
	out_poly.polygon = *poly;
	

	plane1_output.publish(out_poly);
	delete poly;

}

void MeshBound::publish_proj_pts(pcl::PointCloud<pcl::PointXYZ>::Ptr proj_pts)
{
	cout << "Publishing projected points:" << endl;
	sensor_msgs::PointCloud2 temp2;
	pcl::toROSMsg(*proj_pts, temp2);
	proj_output.publish(temp2);
	//cout << "One projected point: " << proj_pts->points[0].x << "  " 
		//<< proj_pts->points[0].y << "  " << proj_pts->points[0].z << endl;

}
