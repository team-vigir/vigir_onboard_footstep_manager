//ros includes
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_lib_io.h>

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
		void listen();

	private:
	        void init_input_topic();
		void init_template_marker();
		void convert_cloud(const sensor_msgs::PointCloud2::ConstPtr& msg);

		ros::NodeHandle n;
		ros::Subscriber cloud_input;
		ros::Publisher mesh_output;
		string in_topic_name;
		string out_topic_name;
		string f_name;
		string f_path;
		visualization_msgs::Marker template_marker;
		vector<visualization_msgs::Marker> markers;
		
};

pcl::PolygonMesh::Ptr mk_mesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
void writeBinarySTL(const char* f_name, const pcl::PolygonMesh& mesh);

int main (int argc, char** argv){	
	//initalize the node
	ros::init(argc, argv, "greedy_proj");
	
	MeshMaker converter;
	converter.listen();

	return 0;
}

pcl::PolygonMesh::Ptr mk_mesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	
	// Load input file into a PointCloud<T> with an appropriate type
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PCLPointCloud2 cloud_blob;
	//pcl::io::loadPCDFile ("bun0.pcd", cloud_blob);
	//pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
	//* the data should be available in cloud

	// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud);
	n.setInputCloud (cloud);
	n.setSearchMethod (tree);
	n.setKSearch (20);
	n.compute (*normals);
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud (cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh::Ptr triangles (new pcl::PolygonMesh);

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius (0.1);

	// Set typical values for the parameters
	gp3.setMu (4);
	gp3.setMaximumNearestNeighbors (100);
	gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
	gp3.setMinimumAngle(M_PI/18); // 10 degrees
	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud (cloud_with_normals);
	gp3.setSearchMethod (tree2);
	gp3.reconstruct (*triangles);

	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();
	
	//Save to file
    //string f_name = "Meshy2.stl";

    //pcl::io::savePolygonFileSTL(f_path.c_str(), triangles);
	//writeBinarySTL(f_path.c_str(), triangles);
	
	//Visualize
	//publish_marker(f_name);

	// Finish
	return triangles;
}

MeshMaker::MeshMaker()
{
    cout << "Currently, this program has minimal validation. Please input"
        << "\nvalid filenames and locations when prompted." << endl;
	string input;
	init_input_topic();

	cout << "Setting Greedy Projection Algorithm's parameters"
	       << "to defaults." << endl;
	
	cout << "Please input an output topic name (1 for /mesh_marker): ";
	cin >> input;
	if (input == "1"){
		out_topic_name = "/mesh_marker";
	} else {
		out_topic_name = input;
	}

	cout << "Please input the output filename of the mesh (1 - test_mesh.stl): ";
	cin >> f_name;
	if (input == "1"){
		f_name = "test_mesh.stl";
	} else {
		f_name = input;
	}

    //Set the saving location of the mesh (hullify package)
	f_path = string(getenv("HOME")) + "/ros/local_cat_ws/src/hullify/meshes/" + f_name;
	
    //Initialize template
	init_template_marker();

	//Initialize publisher and subscriber and begin waiting
	cloud_input = n.subscribe(in_topic_name, 10, &MeshMaker::convert_cloud, this);
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

//Preconditions: f_name (the filename for the mesh), must be
//  set prior to this call.
void MeshMaker::init_template_marker()
{
    cout << "template_marker.header.frame_id hardcoded to 'world' in init_template_marker()" << endl
		<<"\tThis implies the flor system or a bag file must be running." << endl;
	template_marker.header.frame_id = "/world";
	template_marker.header.stamp = ros::Time::now();
    template_marker.action = visualization_msgs::Marker::ADD;
	template_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    template_marker.ns = "temp_namespace";
	template_marker.id = 0;
	
    template_marker.pose.position.x = 1;    //Where it goes
	template_marker.pose.position.y = 1;
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
	string mesh_loc = "package://hullify/meshes/" + f_name;
	cout << "Mesh_loc: " << mesh_loc << endl;
	template_marker.mesh_resource = mesh_loc;
	template_marker.mesh_use_embedded_materials = false;
}

void MeshMaker::listen()
{
	/*ros::Rate freq(1);
	while(ros::ok()){
		cout << "Publishing mesh..." << endl;
		if (markers.size() == 0){
		       //No meshes to publish

		} else if (markers.size() == 1){
			//Just 1 mesh to publish
			mesh_output.publish(template_marker);

		} else {
			//Potentially Many meshes to publish
			//mesh_array_pub.publish(
			cout << "Mesh array functionality not yet supported." << endl;
		}


		freq.sleep();
	}*/
	cout << "Spinning!" << endl;
	ros::spin();

}

void writeBinarySTL(const char* f_name, const pcl::PolygonMesh& mesh)
{
	vtkSmartPointer<vtkPolyData> poly_data = vtkSmartPointer<vtkPolyData>::New();
	pcl::io::mesh2vtk(mesh, poly_data);

	vtkSmartPointer<vtkSTLWriter> poly_writer = vtkSmartPointer<vtkSTLWriter>::New();
	poly_writer->SetInput(poly_data);
	poly_writer->SetFileName(f_name);
	poly_writer->SetFileTypeToBinary();
	poly_writer->Write();

	return;
}


void MeshMaker::convert_cloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	std::cout << "Pointcloud received. Enter input to continue:" << std::endl;
//	string nothing;
//	cin >> nothing;	
	//Convert PointCloud2 message to PCL's PointCloud<XYZ>
	pcl::PointCloud<pcl::PointXYZ>::Ptr intermediate_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	sensor_msgs::PointCloud2 temp_cloud = *msg;
	cout << "MeshMaker::convert_cloud - Why does callback need const message? Will it change what's in the topic if we non-const it?" << endl;
	pcl::moveFromROSMsg(temp_cloud, *intermediate_cloud);

	//Call prebuilt functionality
	pcl::PolygonMesh::Ptr out_poly = mk_mesh(intermediate_cloud);

	//Save the mesh
	writeBinarySTL(f_path.c_str(), *out_poly);
			 
	//Update the mesh marker array
    	if (markers.size() < 1){
    	    markers.push_back(template_marker);
	} else {
    	    markers[0] = template_marker;
	}

	mesh_output.publish(template_marker);    
}
