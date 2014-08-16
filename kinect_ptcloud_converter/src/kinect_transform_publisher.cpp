#include <tf/transform_broadcaster.h>

#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>

using std::cout;
using std::cin;
using std::endl;
using std::vector;
using std::string;

tf::Transform mk_kinect_transform(double kx, double ky, double kz);
void publish_kinect_transform(const string& world_frame, const string& camera_base_frame, tf::Transform& ktransform);

int main(int argc, char** argv){
	ros::init(argc, argv, "kinect_transform_publisher");
	ros::NodeHandle nh;

	cout << "Kinect Transform broadcaster online!" << endl;
	string world_frame = "/adept_combined";
	string camera_base_frame = "/camera_link";
	double kinect_pos_x = -0.09;
	double kinect_pos_y = -0.22;
	double kinect_pos_z = 0.12;

	tf::Transform ktransform = mk_kinect_transform(kinect_pos_x, kinect_pos_y, kinect_pos_z);

	ros::Rate update_freq(30);
	while(ros::ok()){
		publish_kinect_transform(world_frame, camera_base_frame, ktransform);
		update_freq.sleep();
	}

	return 0;

}
tf::Transform mk_kinect_transform(double kx, double ky, double kz)
{
	tf::Transform ktransform;
	ktransform.setOrigin( tf::Vector3(kx, ky, kz) );
	//tf::Vector3 origin = ktransform.getOrigin();
  	cout << "Kinect Origin: " << kx << " " << ky << " " << kz << endl;
  	
  	double qx = 0, qy = 0, qz = 0, qw = 1;
  	tf::Quaternion q(qx, qy, qz, qw);	//(0,0,0,1) is the null transform
  	ktransform.setRotation(q);
  	//tf::Quaternion pose = ktransform.getRotation();
  	cout << "Kinect Orientation: x: " << qx << " y: " << qy << " z: " << qz << " w: " << qw << endl;

  	return ktransform;
}

//This function will broadcast a suitable transform between
//	the adept arm and the location/orientation of the kinect
void publish_kinect_transform(const string& world_frame, const string& camera_base_frame, tf::Transform& ktransform)
{
	static tf::TransformBroadcaster kinect_broadcaster;

  	tf::StampedTransform stamped_ktransform(ktransform, ros::Time::now(), world_frame, camera_base_frame); 
  	kinect_broadcaster.sendTransform(stamped_ktransform);
  	//cout << "Transform between " << world_frame << " and " << camera_base_frame << " published." << endl;
}