#include <string>

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
 
using std::string;

class Ptcloud_transform{
	public:
		Ptcloud_transform(ros::NodeHandle, char*, char*, char*);

		void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in);
		void print_scan_info(const sensor_msgs::LaserScan::ConstPtr& scan_in);
		bool is_pub_time(const sensor_msgs::LaserScan::ConstPtr& scan_in);
		ros::NodeHandle n;
		laser_geometry::LaserProjection projector;
		tf::TransformListener listener;
		message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub;
		tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier;
		ros::Publisher scan_pub;

	private:
		string fixed_frame;
		string output_topic;
		string src_topic;
		sensor_msgs::PointCloud composite;
		long pub_seq;	//The next sequence number to publish at
		int seq_inc;	//The number of messages to complete a full scan.
};

Ptcloud_transform::Ptcloud_transform(ros::NodeHandle in, char* fixed_frame_i, char* src_topic_i, char* output_topic_i):
	n(in), fixed_frame(fixed_frame_i), output_topic(output_topic_i),
	src_topic(src_topic_i), laser_sub(n, src_topic_i, 10),
	laser_notifier(laser_sub, listener, fixed_frame_i, 10) 
{
	pub_seq = -1;
	seq_inc = 0;
	long size = 200000;
	std::cout << "Topic str: " << src_topic << std::endl;
	std::cout << "fixed frame: " << fixed_frame << std::endl;
	std::cout << "Resizing composite vector: " << size << std::endl;
	
	composite.points.reserve(size);

	laser_notifier.registerCallback(
		boost::bind(&Ptcloud_transform::scanCallback, this, _1));
	laser_notifier.setTolerance(ros::Duration(0.01));
	scan_pub = n.advertise<sensor_msgs::PointCloud>(output_topic.c_str(), 1);
}

void Ptcloud_transform::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	print_scan_info(scan_in);

	sensor_msgs::PointCloud cloud;
	try
	{
		projector.transformLaserScanToPointCloud(
			fixed_frame.c_str(), *scan_in, cloud, listener);

	} catch (tf::TransformException& e){
		std::cout << "Hi jack" << std::endl;
		std::cout << e.what();
		return;
	}

	//Work with cloud
/*	std::cout << "Pts:" << std::endl;
	for (int i = 0; i < 12; ++i){
		std::cout << "x: " << cloud.points[i].x << " y: " << cloud.points[i].y << " z: " << cloud.points[i].z << std::endl;
	}
*/
	//Add to the composite
/*	geometry_msgs::Point32 temp;
	temp.x = x;
	x += 0.05;
	temp.y = 0;
 	temp.z = 0;
	composite.points.push_back(temp);
*/
	//Append to the composite
	if (composite.points.size() == 0){
		composite = cloud;
	} else {
		composite.points.insert(composite.points.begin(), cloud.points.begin(), cloud.points.end());
	}

	std::cout << "Size of new cloud: " << cloud.points.size() << std::endl;
	//Publish the composite:TODO Improve complexity
	//cloud.points = composite.points;
//	if (is_pub_time(scan_in))
		scan_pub.publish(composite);
}

//Description: Determines if the laser has completed
//	a full revoution through the scene. 
//Assumptions: The max and min angle of the scan
//	and the increment do not change.
//Returns: true if yes and false if no.
bool Ptcloud_transform::is_pub_time(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	if (scan_in->header.seq == pub_seq){
		pub_seq += seq_inc;
		std::cout << "PUBLISHING." << std::endl << std::endl;
		return true;

	} else if (pub_seq == -1){
		seq_inc = (scan_in->angle_max - scan_in->angle_min) / scan_in->angle_increment;
		std::cout << "seq_inc: " << seq_inc << std::endl;
		pub_seq = scan_in->header.seq + seq_inc;
	}

	return false;
}


void Ptcloud_transform::print_scan_info(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	using std::cout;
	using std::endl;
	cout << "Printing laser data." << endl;
	cout << "\tHeader data:" << endl;
	cout << "\t\tseq: " << scan_in->header.seq << endl;
	cout << "\t\ttime: " << scan_in->header.stamp << endl;
	cout << "\t\tframe: " << scan_in->header.frame_id << endl;
	cout << "\tScan data:" << endl;
	cout << "\t\tangle_min: " << scan_in->angle_min << endl;
	cout << "\t\tangle_max: " << scan_in->angle_max << endl;
	cout << "\t\tangle_increment: " << scan_in->angle_increment << endl;
	cout << "\t\tscan_time: " << scan_in->scan_time << endl;
	cout << endl;

}

int main(int argc, char** argv){
	ros::init(argc, argv, "hullify_node");
	ros::NodeHandle n;	
	//char ff[] = "base_link";
	Ptcloud_transform* trans;	

	if (argc > 1){
		if (argc != 4){
			std::cout << "Argument usage: exec src_topic output_topic_name base_frame" << std::endl;
			std::cout << "The first two arguments should be slash prefaced. The third should not be." << std::endl;
			return 1;
		} else {
			trans = new Ptcloud_transform(n, argv[3], argv[1], argv[2]);
		}
	} else {
		//Use defaults
		char output_topic[] = "/rendered_cloud";
		char scan_src[] = "/laser/scan";	
		char ff[] = "base_link";
		trans = new Ptcloud_transform(n, ff, scan_src, output_topic);

	}

	//laser_geometry::LaserProjection projector;
	//tf::TransformListener listener;
	
	ros::spin();

	return 0;
}
