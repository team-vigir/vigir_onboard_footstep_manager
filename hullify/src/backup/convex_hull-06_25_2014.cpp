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
//#include <pcl/surface/convex_hull.h>

//From Tutorial
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
//End tutorial

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

//pcl::PolygonMesh::Ptr 
void mk_mesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
void writeBinarySTL(const char* f_name, const pcl::PolygonMesh& mesh);
void qhull_save_file(string fname, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
pcl::PolygonMesh::Ptr qhull_read_results(string in_name);
pcl::PolygonMesh::Ptr qhull_mk_mesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

int main (int argc, char** argv){	
	//initalize the node
	ros::init(argc, argv, "greedy_proj");
	
	MeshMaker converter;
	converter.listen();

	return 0;
}
/*
pcl::PolygonMesh::Ptr mk_mesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	
	pcl::PolygonMesh::Ptr output (new pcl::PolygonMesh);
	vector<pcl::Vertices> polygons;
	pcl::PointCloud<pcl::PointXYZ> points;

	pcl::ConvexHull<pcl::PointXYZ> hull;
	hull.setInputCloud(cloud);
	hull.reconstruct(points, polygons);

	output->header.seq = 0;
	output->header.stamp = 1;
	output->header.frame_id = "/world";
	output->polygons.swap(polygons);
	//output->cloud = points;

	//Save to file
    //string f_name = "Meshy2.stl";

    //pcl::io::savePolygonFileSTL(f_path.c_str(), triangles);
	//writeBinarySTL(f_path.c_str(), triangles);
	
	//Visualize
	//publish_marker(f_name);

	// Finish
	return output;
}
*/

//pcl::PolygonMesh::Ptr 
void mk_mesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), 
                                      cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
	//Obtain the cloud
	reader.read("table_scene_mug_stereo_textured.pcd", *cloud);
	cout << "Read in table scene." << endl;

      // Build a filter to remove spurious NaNs
      pcl::PassThrough<pcl::PointXYZ> pass;
      pass.setInputCloud (cloud);
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (0, 1.1);
      pass.filter (*cloud_filtered);
      std::cerr << "PointCloud after filtering has: "
                << cloud_filtered->points.size () << " data points." << std::endl;

      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      // Create the segmentation object
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      // Optional
      seg.setOptimizeCoefficients (true);
      // Mandatory
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setDistanceThreshold (0.01);

      seg.setInputCloud (cloud_filtered);
      seg.segment (*inliers, *coefficients);
      std::cerr << "PointCloud after segmentation has: "
                << inliers->indices.size () << " inliers." << std::endl;

      // Project the model inliers
      pcl::ProjectInliers<pcl::PointXYZ> proj;
      proj.setModelType (pcl::SACMODEL_PLANE);
      proj.setIndices (inliers);
      proj.setInputCloud (cloud_filtered);
      proj.setModelCoefficients (coefficients);
      proj.filter (*cloud_projected);
      std::cerr << "PointCloud after projection has: "
                << cloud_projected->points.size () << " data points." << std::endl;

      // Create a Concave Hull representation of the projected inliers
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::ConcaveHull<pcl::PointXYZ> chull;
      chull.setInputCloud (cloud_projected);
      chull.setAlpha (0.1);
      chull.reconstruct (*cloud_hull);

      std::cerr << "Concave hull has: " << cloud_hull->points.size ()
                << " data points." << std::endl;

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

pcl::PolygonMesh::Ptr qhull_mk_mesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	string f_name = "temp_cloud.txt";
	string res_name = "result.txt";
	qhull_save_file(f_name, cloud);
	
	//Execute qhull command here.
	string command = "qconvex o TO " + res_name;
	command += " < " + f_name;
	system(command.c_str());
	
	return qhull_read_results(res_name);

}

void qhull_save_file(string fname, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	fstream out_file;
	out_file.open(fname.c_str(), fstream::out | fstream::trunc);
	if (!out_file.fail())
		cout << "Qhull input file successfully opened." << endl;

	out_file << "3" << endl;
	out_file << cloud->points.size() << endl;
	for (unsigned int i = 0; i < cloud->points.size(); ++i){
		out_file << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << endl;
	}

	out_file.close();
	if (!out_file.fail()){
		cout << "File closed successfully" << endl;
	}
}

pcl::PolygonMesh::Ptr qhull_read_results(string in_name)
{
	pcl::PolygonMesh::Ptr mesh (new pcl::PolygonMesh);

	//Open file
	fstream in_file;
	in_file.open(in_name.c_str(), fstream::in);
	if (!in_file.fail()){
		cout << "Qhull result file opened." << endl;
	}
	
	//Read vertices
	string temp;
	int pt_cnt;
	int face_cnt;
	in_file >> temp >> pt_cnt >> face_cnt >> temp;	//Dimension data, # points, # facets, # edges
	cout << "Output file contains " << pt_cnt << " points." << endl;
	
	pcl::PointCloud<pcl::PointXYZ> cloud;
	cloud.header.seq = 1;
	cloud.header.stamp = 1;
	cloud.header.frame_id = "0";
	float x, y, z;
	for (int i = 0; i < pt_cnt; ++i){
		in_file >> x >> y >> z;
		cloud.push_back( pcl::PointXYZ(x, y, z));
		//cout << "Added point " << x << "  " << y << "  " << z << endl;
	}

	//Read Faces	
	int v_cnt, j;
	int v;	
	pcl::Vertices cur_face;
	for (int i = 0; i < face_cnt; ++i){
		cur_face.vertices.clear();
		
		in_file >> v_cnt;
		//cout << "\tVertex" << i + 1 << " has " << v_cnt << " vertices: ";
		for (j = 0; j < v_cnt; ++j){
			in_file >> v;
			//cout << v << "  ";
			cur_face.vertices.push_back(v);
		}
		
		//cout << endl;
		mesh->polygons.push_back(cur_face);
	}
	
	//Assemble PolygonMesh
	pcl::toPCLPointCloud2(cloud, mesh->cloud);
	mesh->header.seq = 1;
	mesh->header.stamp = 1;
	mesh->header.frame_id = "/world";

	in_file.close();
	if (!in_file.fail()){
		cout << endl << "Qhull file closed successfully!" << endl;
	}
	return mesh;	
}

void MeshMaker::convert_cloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	std::cout << "Pointcloud received" << std::endl;
	//Convert PointCloud2 message to PCL's PointCloud<XYZ>
	pcl::PointCloud<pcl::PointXYZ>::Ptr intermediate_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	sensor_msgs::PointCloud2 temp_cloud = *msg;
	cout << "MeshMaker::convert_cloud - Why does callback need const message? Will it change what's in the topic if we non-const it?" << endl;
	pcl::moveFromROSMsg(temp_cloud, *intermediate_cloud);

	//Run qhull
	pcl::PolygonMesh::Ptr out_poly = qhull_mk_mesh(intermediate_cloud);

	//Call prebuilt functionality
	//pcl::PolygonMesh::Ptr out_poly; //= mk_mesh(intermediate_cloud);
	//mk_mesh(intermediate_cloud);

	//Delete previous mesh
	//if (prev_mesh != ""){
	//	string command = "rm " + f_path;
	//}

	//Save the mesh
	writeBinarySTL(f_path.c_str(), *out_poly);
			 
	//Update the mesh marker array
    	if (markers.size() < 1){
    	    markers.push_back(template_marker);
	} else {
    	    markers[0] = template_marker;
	}
	
	template_marker.header.stamp = ros::Time::now();
	template_marker.header.seq++;
	mesh_output.publish(template_marker);    
}
