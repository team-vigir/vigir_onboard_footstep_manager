import roslib
import rospy
import tf
from tf import transformations

import geometry_msgs
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from hullify.msg import Mesh_and_bounds
from std_msgs.msg import Header

import numpy
from numpy import array

class openraveIO:
	instances = 0	
	def __init__(self, grasper, final_pose_ref_frame, mesh_ref_frame):
		if self.instances != 0:
			print "Error, openraveIO is a singleton"
			raise Exception()
		else:
			self.instances = 1		
		
		self.grasper = grasper
		self.pub = rospy.Publisher('convex_hull/openrave_grasps', PoseArray)
		self.meshtopic = rospy.Subscriber("convex_hull/openrave_params", Mesh_and_bounds, self.full_info_callback)		
		self.pelvis_listener = tf.TransformListener()
		self.final_pose_ref_frame = final_pose_ref_frame
		self.mesh_ref_frame = mesh_ref_frame
		#self.tROS = tf.()

	def full_info_callback(self, msg):
		print "Got a Mesh_and_bounds_msg!"
		print "Plane1: ", msg.ninety_degree_bounding_planes[0]
		print "Plane2: ", msg.ninety_degree_bounding_planes[1]
	
		self.grasper.replace_target(msg.convex_hull)
	
		self.grasper.find_grasps(msg)

	def publish_poses(self, pose_array):
		pose_array = self.change_poses_ref_frames(pose_array)
		pose_msg = PoseArray()
		pose_msg.poses = pose_array
		pose_msg.header.stamp = rospy.Time.now()
		pose_msg.header.frame_id = self.final_pose_ref_frame
	
		self.pub.publish(pose_msg)

	def change_poses_ref_frames(self, pose_array):
		if self.final_pose_ref_frame != self.mesh_ref_frame:
			#self.tROS.waitForTransform(self.mesh_ref_frame, self.final_pose_ref_frame, pose_array[0].header.stamp, rospy.Duration(1.0))	
			trans, rot = self.get_pelvis_transform()
			print trans, rot		
			for idx, pose in enumerate(pose_array):
				spose = self.transform_pose(pose, trans, rot)
				pose_array[idx] = spose.pose

		return pose_array
	
	def get_pelvis_transform(self):
		freq = rospy.Rate(10)
		while not rospy.is_shutdown():
			try:
				(trans, rot) = self.pelvis_listener.lookupTransform(self.final_pose_ref_frame, self.mesh_ref_frame, rospy.Time(0))
				break
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				print "Lookup exception handled..."
				freq.sleep()

		return trans, rot

	def transform_pose(self, pose, trans, rot):
		xyz = pose.pose.position
		ori = pose.pose.orientation		
		mat44 = self.pelvis_listener.fromTranslationRotation(trans, rot)
		pose44 = numpy.dot(transformations.translation_matrix((xyz.x, xyz.y, xyz.z)), transformations.quaternion_matrix((ori.x, ori.y, ori.z, ori.w)))
		txpose = numpy.dot(mat44, pose44)		
		xyz = tuple(transformations.translation_from_matrix(txpose))[:3]
		quat = tuple(transformations.quaternion_from_matrix(txpose))
		pose.pose.position = geometry_msgs.msg.Point(*xyz)
		pose.pose.orientation = geometry_msgs.msg.Quaternion(*quat)

		return pose

	def TransformToPoseStamped(self, G):
		"""
		Compute geometry_msgs::Pose from 4x4 transform
		G is a 4x4 matrix which is [R t; 0 0 0 1]
		"""
		tj = array( G )[0:3,3]
		q = transformations.quaternion_from_matrix( G )
		orientation = geometry_msgs.msg.Quaternion( q[0], q[1], q[2], q[3] )
		position = geometry_msgs.msg.Point( tj[0], tj[1], tj[2] )
		
		pose = geometry_msgs.msg.Pose( position, orientation )
		header = Header(100, rospy.Time.now(), self.mesh_ref_frame)
		print "time: ", header.stamp		
		spose = PoseStamped(header, pose)
		
		return spose

def testpublisher(raveio):
	print "Running pose publishing test..."	
	pose = raveio.TransformToPoseStamped([[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]])
	pose_array = [pose]
	raveio.publish_poses(pose_array)

	print "Finished test. Were any poses published?"

