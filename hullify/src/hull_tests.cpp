#include "ros/ros.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <cmath>
#include <ctime>
#include <cstdlib>
#include <iostream>
#include <string>

using std::cin;
using std::cout;
using std::endl;
using std::string;

sensor_msgs::PointCloud mk_tetrahed();
sensor_msgs::PointCloud mk_semi_sph(float density);
sensor_msgs::PointCloud mk_line();

int main(int argc, char** argv){
	ros::init(argc, argv, "hull_tests");
	ros::NodeHandle n;

	sensor_msgs::PointCloud test1 = mk_tetrahed();
	sensor_msgs::PointCloud2 tet2;
	sensor_msgs::convertPointCloudToPointCloud2(test1, tet2);

	ros::Publisher p = n.advertise<sensor_msgs::PointCloud2>("/testing/default", 10);
	
	p.publish(tet2);
	sleep(4);

	ros::Duration rate(2);
	float density;
	while (ros::ok()){
		cout << "Please input a density for a semisphere: ";
		//cout << "Please enter 1 to continue (making tetrahedron): ";
		cin >> density;
		
		test1 = mk_semi_sph(density);
		//test1 = mk_line();
		//test1 = mk_tetrahed();
		sensor_msgs::convertPointCloudToPointCloud2(test1, tet2);
		
		p.publish(tet2);
		rate.sleep();
	}

	return 0;
}


sensor_msgs::PointCloud mk_tetrahed()
{
	sensor_msgs::PointCloud tetrahed;
	tetrahed.header.stamp = ros::Time::now();
	tetrahed.header.seq = 1;
	tetrahed.header.frame_id = "/world";
	
	geometry_msgs::Point32 pt;
	pt.x = 0; pt.y = 0; pt.z = 0;
	tetrahed.points.push_back(pt);
	pt.x = 1;
	tetrahed.points.push_back(pt);
	pt.y = 1;
	tetrahed.points.push_back(pt);
	pt.x = 0;
	tetrahed.points.push_back(pt);

	/*for (int i = 0; i < 4; ++i){
		pt = tetrahed.points[i];
		pt.z = 1;

	}*/
	pt.x = 0.5; pt.y = 0.5; pt.z = 1;
	tetrahed.points.push_back(pt);
	pt.z = -1;
	tetrahed.points.push_back(pt);

	return tetrahed;
}


//Parameters: density of 0 means no cloud, density of 1
//	means maximum resolution.
sensor_msgs::PointCloud mk_semi_sph(float density)
{
	sensor_msgs::PointCloud semi_sph;
	semi_sph.header.stamp = ros::Time::now();
	semi_sph.header.seq = 1;
	semi_sph.header.frame_id = "/world";

	float radius = 1;
	float d_theta = .1;	//rad
	float d_phi = .1;	//also rad
	
	geometry_msgs::Point32 cur_pt;
	float j;
	float p;
	srand(time(NULL));
	for (float i = 0; i < 3.14159265; i += d_phi){
		for (j = 0; j < 3.14159265; j += d_theta){
			//Place a point here if we can
			p = ((rand() % 100) + 1) / 100.0;
			if (p <= density){
				cur_pt.x = radius * sin(i) * cos(j);
				cur_pt.y = radius * sin(i) * sin(j);
				cur_pt.z = radius * cos(i);

				semi_sph.points.push_back(cur_pt);
			}
		}
	}

	return semi_sph;
}


sensor_msgs::PointCloud mk_line()
{
	//Generate a slope
	geometry_msgs::Point32 slope;
	slope.x = rand() % 5;
	slope.y = rand() % 5;
	slope.z = rand() % 5;

	//start pt
	geometry_msgs::Point32 start;
	start.x = (rand() % 5) / (double (rand()%4) + 1);
	start.y = (rand() % 6) / (double (rand()%5) + 1);
	start.z = (rand() % 7) / (double (rand()%6) + 1);

	sensor_msgs::PointCloud line;
	line.header.stamp = ros::Time::now();
	line.header.seq = 1;
	line.header.frame_id = "/world";
	
	geometry_msgs::Point32 temp;
	for (int i = 0; i < 4; ++i){
		temp.x = start.x + slope.x * i;
		temp.y = start.y + slope.y * i;
		temp.z = start.z + slope.z * i;
		line.points.push_back(temp);
	}

	return line;
}
