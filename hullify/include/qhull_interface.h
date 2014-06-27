#ifndef QHULL_INTERFACE_H
#define QHULL_INTERFACE_H

#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

#include <cstdlib>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>

using std::string;
using std::vector;
using std::cin;
using std::cout;
using std::endl;
using std::fstream;

class Qhull_int {
    public:
        Qhull_int();
        ~Qhull_int();
        
        pcl::PolygonMesh::Ptr mk_mesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    private:
        void qhull_save_file(string fname, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        pcl::PolygonMesh::Ptr qhull_read_results(string in_name);

        string in_file_name;    //The name of the input file to qHull
        string res_file_name;   //The name of the output file (OFF format)

};

#endif
