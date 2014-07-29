#include "hullify_view.h"

//Simply removes all meshes
Hullify_View::~Hullify_View()
{
	map<string, string>::iterator cur_mesh = meshes.begin();
	for (; cur_mesh != meshes.end(); cur_mesh++){
		rm_prev_mesh(cur_mesh);
	}
}

Hullify_View::Hullify_View(){
	cout << "Default constructor called: /world as visualization_ref_frame and no prefix." << endl;
	prefix = "";
	visualization_ref_frame = new string("/world");

}

Hullify_View::Hullify_View(string in_prefix, string* ff_in){
	prefix = in_prefix;
	visualization_ref_frame = ff_in;

	common_constructor();
}

void Hullify_View::common_constructor()
{
	cout << "Creating view for node." << endl;
	meshes.clear();
	topics.clear();
	init_template_marker();
	get_mesh_folder_path();
}

void Hullify_View::get_mesh_folder_path()
{
	mesh_folder_path = ros::package::getPath("hullify");
	if (mesh_folder_path == ""){
		ROS_ERROR("Error: Could not retrieve package hullify's location (in Hullify_View). Terminating node.");
		exit(1);
	}

	mesh_folder_path += "/meshes/";
}

void Hullify_View::init_template_marker()
{
	template_marker.header.frame_id = *visualization_ref_frame;
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

/****** Map Material
***************************/
bool Key_Compare::operator()(string key1, string key2)
{
	if (strcmp(key1.c_str(), key2.c_str()) < 0){
		return true;
	}

	return false;

}

//Function: add_mesh_topic()
//Description: This function creates bookkeeping and
//	publishing information for a new mesh resource.
void Hullify_View::add_mesh_topic(string base_name)
{
	if (base_name == ""){
		cout << "Error: The name of the new mesh topic is empty." << endl;
		return;
	}

	pair<string, string> new_mesh(base_name, "");

	pair<map<string, string>::iterator, bool> ret_val = meshes.insert(new_mesh);
	
	if (!ret_val.second){
		cout << "Could not add mesh topic " << base_name << ". Topic already exists in Hullify_View's meshes." << endl;
	}

	visualization_msgs::Marker marker_type;
	add_topic_no_queue(base_name, marker_type);

}

//Function: publish_mesh()
//Description: Deletes a previous mesh if it exists
//	saves the mesh file, publishes the mesh and
//	updates the previous mesh log
void Hullify_View::publish_mesh(string base_name, pcl::PolygonMesh::Ptr new_mesh)
{
	map<string, string>::iterator mesh_info = meshes.find(base_name);
	if (mesh_info == meshes.end()){
		cout << "Could not find mesh topic to publish to (" << base_name
			 << "). Skipping publishing." << endl;
		return;
	}

	string full_mesh_path = mk_mesh_path(base_name);
	rm_prev_mesh(mesh_info);
	write_binary_stl(full_mesh_path, *new_mesh);

	visualization_msgs::Marker mesh_msg = mk_mesh_msg(full_mesh_path);
	publish(base_name, mesh_msg);

	mesh_info->second = full_mesh_path;

}

void Hullify_View::rm_prev_mesh(map<string, string>::iterator& mesh_info)
{
	if (mesh_info->second != ""){
		cout << "Deleting previous mesh: " << mesh_info->first << endl;
		string command = "rm " + mesh_info->second;
		system(command.c_str()); 

	} else {
		cout << "No previous mesh to delete." << endl;

	}
}

//Description: Prepends the mesh folder location
//	and appends time stamp information to a base_name
//	to make unique mesh name.
string Hullify_View::mk_mesh_path(string base_name)
{
	string path = mesh_folder_path + base_name;
	path += boost::lexical_cast<std::string>(time(NULL));
	path += ".stl";

	return path;
}

void write_binary_stl(string full_abs_path, const pcl::PolygonMesh& mesh)
{
	//cout << "Writing mesh to " << full_abs_path << endl;
	vtkSmartPointer<vtkPolyData> poly_data = vtkSmartPointer<vtkPolyData>::New();
	pcl::io::mesh2vtk(mesh, poly_data);

	vtkSmartPointer<vtkSTLWriter> poly_writer = vtkSmartPointer<vtkSTLWriter>::New();
	poly_writer->SetInput(poly_data);
	poly_writer->SetFileName(full_abs_path.c_str());
	poly_writer->SetFileTypeToBinary();
	int ret = poly_writer->Write();
	if (ret != 1){
		cout << "Could not write to STL file in write_binary_stl()" << endl;
	}

	return;
}

void Hullify_View::add_polygon_header(geometry_msgs::PolygonStamped& poly)
{
	poly.header.stamp = ros::Time::now();
	poly.header.seq = 1;
	poly.header.frame_id = *visualization_ref_frame;
}

//This function is for debugging purposes,
//	it makes a disposable marker for showing
//	the centroid. It is inefficient.
visualization_msgs::Marker Hullify_View::mk_pt_msg(Eigen::Vector3d pt)
{
	//Make a new marker
	visualization_msgs::Marker center_marker = template_marker;
	center_marker.type = visualization_msgs::Marker::SPHERE;
	center_marker.pose.position.x = pt[0];
	center_marker.pose.position.y = pt[1];
	center_marker.pose.position.z = pt[2];
	center_marker.color.b = 1;
	center_marker.color.g = 0;
	center_marker.id = 1;
	
	//Set the size of the marker.
	center_marker.scale.x = .1;
	center_marker.scale.y = .1;
	center_marker.scale.z = .1;

	return center_marker;
}

visualization_msgs::Marker Hullify_View::mk_vector_msg(Eigen::Vector3d vector_rep)
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
	temp.x = vector_rep[0]; temp.y = vector_rep[1]; temp.z = vector_rep[2];
	normal_marker.points.push_back(temp);

	return normal_marker;
}

//Parameters: location - This string is the path to the
//	file on the local system. It must be absolute.
visualization_msgs::Marker Hullify_View::mk_mesh_msg(string location)
{
	visualization_msgs::Marker mesh_msg = template_marker;
	mesh_msg.header.stamp = ros::Time::now();
	mesh_msg.mesh_resource = "file://" + location;

	return mesh_msg;
}

//Function: mk_square_plane_rep()
//Description: Given a central point, a distal point, and
//	a normal vector, this function will draw a square that
//	represents a portion of the plane given by the central
//	point and normal.
geometry_msgs::Polygon Hullify_View::mk_square_plane_rep(pcl::PointXYZ center, pcl::PointXYZ max_pt, Eigen::Vector3d normal)
{
	geometry_msgs::Polygon out_poly;

	//Make maximum and scale for viewing
	Eigen::Vector3d max((max_pt.x - center.x), (max_pt.y - center.y), (max_pt.z - center.z));
	max = max * 1.5;

	//Make a square where orth points to another vertex
	Eigen::Vector3d orth = max.cross(normal);
	orth = (orth / orth.norm()) * max.norm();

	geometry_msgs::Point32 temp;
	temp.x = center.x + max[0]; temp.y = center.y + max[1]; temp.z = center.z + max[2];
	out_poly.points.push_back(temp);
	temp.x = center.x + orth[0]; temp.y = center.y + orth[1]; temp.z = center.z + orth[2];
	out_poly.points.push_back(temp);
	temp.x = center.x - max[0]; temp.y = center.y - max[1]; temp.z = center.z - max[2];
	out_poly.points.push_back(temp);
	temp.x = center.x - orth[0]; temp.y = center.y - orth[1]; temp.z = center.z - orth[2];
	out_poly.points.push_back(temp);

	return out_poly;
}

//This function takes a bounding line, which is presumed
//	to pass through the centroid, and a longest distance
//	vector to produce a plane representation
//Parameters: line_pt_to_bound points orthogonal to slope 
//		and the plane's normal. It points to a boundary point in plane.
geometry_msgs::Polygon Hullify_View::mk_plane_rep_from_bounding_line(Eigen::Vector3d line_pt, Eigen::Vector3d slope, Eigen::Vector3d line_pt_to_bound)
{
	pcl::PointXYZ center;
	center.x = line_pt[0];
	center.y = line_pt[1];
	center.z = line_pt[2];

	geometry_msgs::Polygon out_poly;
	
	//Make slope the same magnitude as line_pt_to_bound
	slope *= (line_pt_to_bound.norm() / slope.norm());

	//Make a square where orth points to another vertex
	geometry_msgs::Point32 temp;
	temp.x = center.x + slope[0]; temp.y = center.y + slope[1]; temp.z = center.z + slope[2];
	out_poly.points.push_back(temp);
	
	temp.x -= (2 * slope[0]); temp.y -= (2 * slope[1]); temp.z -= (2 * slope[2]);
	out_poly.points.push_back(temp);

	temp.x += line_pt_to_bound[0]; temp.y += line_pt_to_bound[1]; temp.z += line_pt_to_bound[2];
	out_poly.points.push_back(temp);

	temp.x += (2 * slope[0]); temp.y += (2 * slope[1]); temp.z += (2 * slope[2]);
	out_poly.points.push_back(temp);

	return out_poly;
}

geometry_msgs::PoseStamped Hullify_View::mk_pose_msg(Eigen::Quaterniond quat, Eigen::Vector3d pose_position)
{
	geometry_msgs::PoseStamped pose_msg;
	pose_msg.header.frame_id = *visualization_ref_frame;
	pose_msg.header.stamp = ros::Time::now();

	geometry_msgs::Point pose_location;
	pose_location.x = pose_position[0];
	pose_location.y = pose_position[1];
	pose_location.z = pose_position[2];

	pose_msg.pose.position = pose_location;

	pose_msg.pose.orientation.w = quat.w();
	pose_msg.pose.orientation.x = quat.x();
	pose_msg.pose.orientation.y = quat.y();
	pose_msg.pose.orientation.z = quat.z();

	return pose_msg;
}
