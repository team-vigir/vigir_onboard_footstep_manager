//ros includes
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>

#include "pcl/ros/conversions.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/project_inliers.h>
//#include <pcl/surface/convex_hull.h>

//From Tutorial
/*#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>*/
//End tutorial

#include <Eigen/Dense>

#include "qhull_interface.h"
#include "mesh_bound.h"

#include <ctime>
#include <cstdlib>
#include <string>
#include <vector>
using std::string;
using std::vector;
using std::cout;
using std::cin;
using std::endl;

class MeshMaker{
	public:
		MeshMaker();
		~MeshMaker();
		void listen();

	private:
		void init_input_topic();
		void init_template_marker();
		void convert_cloud(const sensor_msgs::PointCloud2::ConstPtr& msg);
        bool get_cloud(const sensor_msgs::PointCloud2::ConstPtr& msg, pcl::PointCloud<pcl::PointXYZ>::Ptr intermediate_cloud);
        bool is_valid_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
		
		ros::NodeHandle n;
		ros::Subscriber cloud_input;
		ros::Publisher mesh_output;
		string fixed_frame;
		string in_topic_name;
		string out_topic_name;
		string base_name;   //Default, test_mesh
		string prev_name;  //The name of the last saved mesh file (for deletion purposes)
		string f_path;  //Path to the hullify package on Eva
		vector<visualization_msgs::Marker> markers;
		visualization_msgs::Marker template_marker;
		Qhull_int qhull;
		MeshBound* bounds;

};

//pcl::PolygonMesh::Ptr mk_mesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
void writeBinarySTL(const char* f_name, const pcl::PolygonMesh& mesh);
double pt_dist(pcl::PointXYZ pt1, pcl::PointXYZ pt2);

int main (int argc, char** argv){	
	//initalize the node
	ros::init(argc, argv, "greedy_proj");

	MeshMaker converter;
	converter.listen();

	return 0;
}
/*
//Ought to work. Untested because qhull libraries were insufficient.
//  Requires precisely Qhulllib5 (2011.1)
pcl::PolygonMesh::Ptr mk_mesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

//See PCL Concave Hull tutorial for filters
cout << "No filters for pointcloud. Is it filtered?" << endl;

pcl::PolygonMesh::Ptr output (new pcl::PolygonMesh);
vector<pcl::Vertices> polygons;
pcl::PointCloud<pcl::PointXYZ> points;

pcl::ConvexHull<pcl::PointXYZ> hull;
hull.setInputCloud(cloud);
hull.reconstruct(output);

output->header.seq = 0;
output->header.stamp = 1;
output->header.frame_id = "/world";

// Finish
return output;
}
*/


MeshMaker::MeshMaker()
{
	prev_name = "";
	fixed_frame = "/world";
	bounds = new MeshBound(fixed_frame);

	cout << "Currently, this program has minimal validation. Please input"
		<< "\n\tvalid filenames and locations when prompted." << endl;
	string input;
	init_input_topic();

	cout << "Please input an output topic name (1 for /mesh_marker): ";
	cin >> input;
	if (input == "1"){
		out_topic_name = "/mesh_marker";
	} else {
		out_topic_name = input;
	}

	cout << "Please input the base output filename of the mesh (1 - test_mesh): ";
	cin >> base_name;
	if (input == "1"){
		base_name = "test_mesh";
	}

	//Set the saving location of the mesh (hullify package)
	f_path = string(getenv("HOME")) + "/ros/local_cat_ws/src/hullify/meshes/";

	//Initialize template
	init_template_marker();

	//Initialize publisher and subscriber and begin waiting
	cloud_input = n.subscribe(in_topic_name, 1, &MeshMaker::convert_cloud, this);
	mesh_output = n.advertise<visualization_msgs::Marker>(out_topic_name.c_str(), 5);

	cout << "Publisher and Subscriber Created." << endl;
	cout << "in_topic: " << in_topic_name << " out_topic: " 
		<< out_topic_name << endl;
}

void MeshMaker::init_input_topic()
{
	string input;
	while(1){
		cout << "What is the pointcloud input topic for this meshing node?"
			<< "\n\t0 - new\n\t1 - /testing/default"
			<< "\n\t2 - /flor/worldmodel/ocs/dist_query_pointcloud_result: ";
		cin >> input;

		if (input == "0"){
			cout << "Please input the topic name: ";
			cin >> in_topic_name;

		} else if (input == "1"){
			in_topic_name = "/testing/default";

		} else if (input == "2"){
			in_topic_name = "/flor/worldmodel/ocs/dist_query_pointcloud_result";

		} else {
			cout << "Invalid entry (must be 0, 1, or 2)" << endl;
			continue;
		}

		break;
	}
}

MeshMaker::~MeshMaker()
{
	//If there is any mesh file left, delete it
	if (prev_name != ""){
		cout << "Deleting: " << prev_name << endl;
		string command = "rm " + f_path + prev_name;
		system(command.c_str());
	}
}

//Preconditions: f_name (the filename for the mesh), must be
//  set prior to this call.
void MeshMaker::init_template_marker()
{
	cout << "Warning: template_marker.header.frame_id hardcoded to '/world'" << endl;
	template_marker.header.frame_id = fixed_frame;
	template_marker.header.stamp = ros::Time::now();
	template_marker.header.seq = 1;
	template_marker.action = visualization_msgs::Marker::ADD;
	template_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
	template_marker.ns = "temp_namespace";
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

//Simply spin and wait for callback requests
//We are using the singly-threaded spin() function, it
//	goes to the global callback queue, finds the oldest
//	callback and executes it.
//A multithreaded option is available, but it would take up
//	system resources we just don't need. Little gain.
void MeshMaker::listen()
{
	cout << "Awaiting pointclouds..." << endl;
	ros::spin();
	//ros::MultiThreadedSpinner spinner(4); //Note: queue size is 1, if you uncomment these lines, that must be changed as well.
	//spinner.spin();

}

void writeBinarySTL(const char* f_name, const pcl::PolygonMesh& mesh)
{
	vtkSmartPointer<vtkPolyData> poly_data = vtkSmartPointer<vtkPolyData>::New();
	pcl::io::mesh2vtk(mesh, poly_data);

	vtkSmartPointer<vtkSTLWriter> poly_writer = vtkSmartPointer<vtkSTLWriter>::New();
	poly_writer->SetInput(poly_data);
	poly_writer->SetFileName(f_name);
	poly_writer->SetFileTypeToBinary();
	int ret = poly_writer->Write();
	if (ret != 1){
		cout << "Could not write to STL file in writeBinarySTL()" << endl;
	}

	return;
}


//Function: convert_cloud
//Description: The workhorse; this function receives pointcloud
//	requests, calls qhull, updates the the mesh markers and 
//	publishes the result.
//Parameters: A pointcloud obtained from in_topic_name (set in constructor)
//Postconditions: A convex hull has been generated and published.
//	goes back to listen()
//Assumptions: This node handles one end to end computational process:
//	one callback.
void MeshMaker::convert_cloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	std::cout << "Pointcloud received" << std::endl;

	//Convert PointCloud2 message to PCL's PointCloud<PointXYZ>
	pcl::PointCloud<pcl::PointXYZ>::Ptr intermediate_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	if (!get_cloud(msg, intermediate_cloud)){
        	//Invalid cloud, return nothing.
        	return;
    	}

	//Run qhull (or see comments just below)
	pcl::PolygonMesh::Ptr out_poly = qhull.mk_mesh(intermediate_cloud);

	//In a system where qHull (libqhull5) is v2011.1, the below should work (untested).
	//pcl::PolygonMesh::Ptr out_poly = mk_mesh(intermediate_cloud);
	
	//Get plane and centroid
	bounds->set_input_cloud(intermediate_cloud);
	bounds->find_centroid();
	bounds->find_plane();
	bounds->publish_plane1();
	bounds->publish_centroid();
	pcl::ModelCoefficients::Ptr plane (new pcl::ModelCoefficients);
	*plane = bounds->get_plane1();

	//Delete previous mesh
	if (prev_name != ""){
		string command = "rm " + f_path + prev_name;
		/*int ret =  */system(command.c_str());
		//cout << "Return code of delete call: " << ret << endl;
		prev_name = "";
	}/* else {
		cout << "Did not run rm on stl mesh file." << endl;
		cout << "Prev_name: '" << prev_name << "'" << endl;
	}*/

	//Save the mesh
	prev_name = base_name + boost::lexical_cast<std::string>(time(NULL)) + ".stl";
	string temp = f_path + prev_name;
	writeBinarySTL(temp.c_str(), *out_poly);

	//Update the mesh marker array
	if (markers.size() < 1){
		markers.push_back(template_marker);
	} else {
		markers[0] = template_marker;
	}

	template_marker.mesh_resource = "package://hullify/meshes/" + prev_name;
	template_marker.header.stamp = ros::Time::now();
	template_marker.header.seq++;
	mesh_output.publish(template_marker);   	
}

bool MeshMaker::get_cloud(const sensor_msgs::PointCloud2::ConstPtr& msg, pcl::PointCloud<pcl::PointXYZ>::Ptr intermediate_cloud)
{
	cout << "MeshMaker::convert_cloud - Why does callback need const message?\n\t"
		<< "Will it change what's in the topic if we non-const it?" << endl;
	sensor_msgs::PointCloud2 temp_cloud = *msg;
	pcl::moveFromROSMsg(temp_cloud, *intermediate_cloud);

    if (!is_valid_cloud(intermediate_cloud)){       
        return false;
    }
    return true;
}

//Determines if the cloud has enough non-coplanar points to describe
//  a convex hull.
bool MeshMaker::is_valid_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    if (cloud->points.size() < 4){
		cout << "Insufficient information in pointcloud request to generate a convex hull.\n\tInput requires at least 4 points." << endl;
        return false;
	}

    long num_pts = cloud->points.size();    
    pcl::PointXYZ test_pts[4];
    test_pts[0] = cloud->points[0];
    test_pts[1] = cloud->points[1];
    geometry_msgs::Point slope;
    slope.x = test_pts[1].x - test_pts[0].x;
    slope.y = test_pts[1].y - test_pts[0].y;
    slope.z = test_pts[1].z - test_pts[0].z;
    
    long i;
    bool success = false;
    double dx, dt;    
    for (i = 2; i < num_pts; i++){
        dx = cloud->points[i].x - test_pts[1].x;
        dt = dx / slope.x;
        if ((test_pts[1].y + slope.y * dt != cloud->points[i].y) ||
            (test_pts[1].z + slope.z * dt != cloud->points[i].z)){
            success = true;
            break;
        }
    }

    if (!success){
        cout << "No non-collinear points exist in input cloud." << endl;        
        return false;
    } else {
        test_pts[2] = cloud->points[i];
    }
    
    //Find non-coplanar points:
    /*success = false;
    for (i++; i < num_pts; ++i){
        
    }*/

    

    return true;
}
