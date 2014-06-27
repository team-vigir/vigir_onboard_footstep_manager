#include "qhull_interface.h"

Qhull_int::Qhull_int()
{
    //Set some file names
    in_file_name = "qhull_in_tempZZZ.txt";
    res_file_name = "qhull_resultsZZZ.txt";
}

Qhull_int::~Qhull_int()
{
    //Destroy the temp file if necessary
    string command = "rm " + in_file_name;
    system(command.c_str());
    command = "rm " + res_file_name;
    system(command.c_str());

    cout << "Deleted temporary qHull IO text files." << endl;
}

pcl::PolygonMesh::Ptr Qhull_int::mk_mesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	//string f_name = "temp_cloud.txt";
	//string res_name = "result.txt";
	qhull_save_file(in_file_name, cloud);
	
	//Execute qhull command here.
	string command = "qconvex o TO " + res_file_name;
	command += " < " + in_file_name;
	system(command.c_str());	

	return qhull_read_results(res_file_name);

}

void Qhull_int::qhull_save_file(string fname, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	fstream out_file;
	out_file.open(fname.c_str(), fstream::out | fstream::trunc);
	if (!out_file.fail())
		cout << "Qhull input file successfully opened." << endl;
    
    //qHull requires the dimension, the number of points, and the points themselves.
	out_file << "3" << endl;
	out_file << cloud->points.size() << endl;
	for (unsigned int i = 0; i < cloud->points.size(); ++i){
		out_file << cloud->points[i].x << " " << cloud->points[i].y 
            << " " << cloud->points[i].z << endl;
	}

	out_file.close();
	if (!out_file.fail()){
		cout << "File closed successfully" << endl;
	}
}

pcl::PolygonMesh::Ptr Qhull_int::qhull_read_results(string in_name)
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
	in_file >> temp >> pt_cnt >> face_cnt >> temp;	//Dimension, # points, # facets, # edges
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
