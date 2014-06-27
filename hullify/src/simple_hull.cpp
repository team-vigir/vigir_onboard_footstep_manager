#include <string>
#include <cstdlib>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/PolygonStamped.h"
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/io/vtk_lib_io.h>//For mesh saving.
#include <pcl/surface/organized_fast_mesh.h>

using std::string;

class Distributor{
	public:
		Distributor();
		Distributor(string cloud_tpk, string hull_tpk);
		void send_cloud(sensor_msgs::PointCloud2& msg);
		void send_hull(geometry_msgs::PolygonStamped& polygon);
	private:
		ros::NodeHandle nh;
		ros::Publisher cloud_pub;
		ros::Publisher hull_pub;
		string cloud_topic, hull_topic; //Output topics
};

Distributor::Distributor()
{
	std::cout << "Error, calling default constructor for Dsitributor." << std::endl;
	exit(2);
}

Distributor::Distributor(string cloud_tpk, string hull_tpk)
{
	cloud_topic = cloud_tpk;
	hull_topic = hull_tpk;
	cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(cloud_topic.c_str(), 10);
	std::cout << "Need hull_pub init and send function creation" << std::endl;
	hull_pub = nh.advertise<geometry_msgs::PolygonStamped>(hull_topic.c_str(), 10);	
}

void Distributor::send_cloud(sensor_msgs::PointCloud2& cloud)
{
	cloud_pub.publish(cloud);
}

void Distributor::send_hull(geometry_msgs::PolygonStamped& polygon)
{
	hull_pub.publish(polygon);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr create_cloud(Distributor& out);
void mk_hull(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in);
void publish_hull();

int main(int argc, char** argv){
	ros::init(argc, argv, "simple_hull");

	Distributor out("simple_cloud", "polygon_hull");

	pcl::PointCloud<pcl::PointXYZ>::Ptr cld = create_cloud(out);

	mk_hull(cld);

	publish_hull();
	std::cout << "Hello Jack." << std::endl;
	ros::spin();
	return 0;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr create_cloud(Distributor& out)
{
	sensor_msgs::PointCloud cloud;
	sensor_msgs::PointCloud2 temp_cloud;
	cloud.header.stamp  = ros::Time();
	cloud.header.frame_id = "world";
	geometry_msgs::Point32 pt;
	pt.x = 0; pt.y = 0; pt.z = 0;
	cloud.points.push_back(pt);
	pt.y = 1;
	cloud.points.push_back(pt);
	pt.x = 1;
	cloud.points.push_back(pt);
	pt.y = 0;
	cloud.points.push_back(pt);
	pt.x = 0.5; pt.y = 0.5; pt.z = 1;
	cloud.points.push_back(pt);

	sensor_msgs::convertPointCloudToPointCloud2(cloud, temp_cloud);
	/*ros::Rate a(1);
	while(ros::ok()){
		out.send_cloud(temp_cloud);
		a.sleep();
	}*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud( new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(temp_cloud, *out_cloud);
	pcl::OrganizedFastMesh<pcl::PointXYZ> mesher;
	//pcl::PointCloud<pcl::PointXYZ>::ConstPtr temp = out_cloud;
	mesher.setInputCloud(out_cloud);

	pcl::PolygonMesh mesh;
	mesher.reconstruct(mesh);
	std::cout << "Just about to make file names... " << std::endl;
	string f_name = "meshy";
	pcl::io::savePolygonFileSTL(f_name, mesh);	
	
	//pcl_conversions::moveToPCL(temp_cloud, *out_cloud);
	
	/*geometry_msgs::Polygon shape;
	geometry_msgs::PolygonStamped out_poly;
	shape.points.swap(cloud.points);
	out_poly.polygon = shape;
	out_poly.header = cloud.header;
	ros::Rate a(1);
	while(ros::ok()){
		out.send_hull(out_poly);
		std::cout << "Please input an x, y and z:" << std::endl;
		std::cin >> pt.x >> pt.y >> pt.z;
		out_poly.polygon.points.push_back(pt);
		
		a.sleep();
	}*/
	
	return out_cloud;
}

void mk_hull(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in)
{
	//pcl::ConvexHull<pcl::PointXYZ> chull;
	//chull.setInputCloud(*cloud_in);
}

void publish_hull()
{

}
