#!/usr/bin/env python
import rospy
from openravepy import *
from openravepy.examples import tutorial_grasptransform
import rospkg
import os
import numpy, time
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray
from hullify.msg import Mesh_and_bounds

import geometry_msgs
import tf
from tf import transformations as tr
import tf_conversions.posemath as pm
from numpy import pi, eye, dot, cross, linalg, sqrt, ceil, size
from numpy import hstack, vstack, mat, array, arange, fabs, zeros
import math

import grasping
import plane_filters
import atlas_and_ik

env = None
gt = None
obj_name = ""
cur_hand = "l_robotiq"
grasp_target = 'grasp_target'
arm_type = "L"
robot = None
pelvis_listener = None
final_pose_frame = ""

#Environment var name->[file_name, name_in_system]
FILE_PATH = 0
ENV_NAME = 1
loaded_hands = {"l_robotiq":['robots/robotiq.dae', '']}

def set_openrave_environment_vars():
	rospack = rospkg.RosPack()
	rave_to_moveit_path = rospack.get_path('rave_to_moveit')
	if os.environ.get("OPENRAVE_DATA", "") != "":
		os.environ["OPENRAVE_DATA"] = rave_to_moveit_path + os.environ["OPENRAVE_DATA"]
	else:
		os.environ["OPENRAVE_DATA"] = rave_to_moveit_path

	#print "set env vars"

def build_environment():
	global env
	global gt
	global robot

	env = Environment()
	#load_hands()
	#env.Load('scenes/test2.env.xml')
	robot = atlas_and_ik.load_atlas(env)
	env.Load('scenes/grasp_target.env.xml')
	target = env.GetKinBody('grasp_target')
	#env.Load('adeptsetup.robot.xml')

	env.SetViewer('qtcoin')
	gt = tutorial_grasptransform.GraspTransform(env,target)
	a1 =  gt.drawTransform(robot.GetTransform(), length=1)
	a2 = gt.drawTransform(robot.GetActiveManipulator().GetEndEffector().GetTransform(), length=1)
	raw_input("Drew robot transform frame. Pausing before leaving scope...")
	#atlas_and_ik.test_transforms(gt)
	

def load_hands():
	global env
	global loaded_hands

	for hand_name in loaded_hands:
		robot = env.ReadRobotXMLFile(loaded_hands[hand_name][FILE_PATH])
		if robot is None:
			print "Cannot load robot: ", loaded_hands[hand_name][FILE_PATH]
		else:
			env.AddRobot(robot)
			loaded_hands[hand_name][ENV_NAME] = robot.GetName()
			print "Loaded ", loaded_hands[hand_name][ENV_NAME]

def query_final_pose_frame():
	global final_pose_frame
	while True:
		suggestion = raw_input("Please input the reference frame of the final pose: \n\t0 - new \n\t1 - /world: ")
		if suggestion == "0":
			final_pose_frame = raw_input("what's your new frame? (please give it a slash prefix")
		elif suggestion == "1":
			final_pose_frame = "/world"

		else:
			continue

		break

#The main function grabs the environment you want, setting the robot as the first
#robot tag put into your env file. Then it sets up the ikfast, the target for the robot
#and the database grasps are store in. Then send to the TransformToPose function
def main(mesh_and_bounds_msg): 
	#If there was a previous object, delete it:
	global env
	global obj_name
	global grasp_target
	global robot
	global ikmodel
	global gt
	
	target = env.GetKinBody(grasp_target)
	print "Got target!"

	gmodel = grasping.GraspingModel(robot,target)
	#print dir(gmodel.grasper)
	#print gmodel.grasper.__class__
	if not gmodel.load():
		params = plane_filters.generate_grasp_params(gmodel, mesh_and_bounds_msg)
		grasps = get_grasps(gmodel, mesh_and_bounds_msg, params, gt)
		if len(grasps) < 2:
			return [geometry_msgs.msg.Pose()]
	
	print len(gmodel.grasps), " Grasps available."
	graspnum = input("Please enter number of valid grasps to return: ")
	validgrasps, validindices = gmodel.computeValidGrasps(returnnum=graspnum)
	print "validgrasps: ", validgrasps
	print "validindices: ", validindices
	basemanip = interfaces.BaseManipulation(robot)
	pose_array = []
	with robot:
		x = 0
		req_idxs = range(0, graspnum)
		while (x < req_idxs) and x < len(validgrasps):
			grasp = validgrasps[x]
			gmodel.setPreshape(grasp)
			T = gmodel.getGlobalGraspTransform(grasp,collisionfree=True)
			p = TransformToPose(T)
			pose_array.append(p)

			x += 1
		
		print "pose_array: ", pose_array

	publish_poses(pose_array, mesh_and_bounds_msg.header.frame_id)

def get_grasps(gmodel, mesh_and_bounds_msg, params, gt):
	grasps = []
	partitioned_rays = partition_rays(mesh_and_bounds_msg, params['approachrays'])

	if sum([len(x) for x in partitioned_rays]) < 1:
		print "Insufficient approach rays generated. How do the planes look? Returning null pose."
		return []

	for rays in partitioned_rays:
		params['approachrays'] = rays
		#atlas_and_ik.visualize_approaches(gt, params)
		gmodel.generate(**params)

		grasps.extend(gmodel.grasps)
		if len(grasps) > 2:
			return grasps

	return grasps
	
def partition_rays(mesh_and_bounds_msg, rays):
	partitioned_rays = []
	sweet_idxs = plane_filters.filter_bounding_planes(rays, mesh_and_bounds_msg.ninety_degree_bounding_planes[0].coef, mesh_and_bounds_msg.ninety_degree_bounding_planes[1].coef, False)

	partitioned_rays.append(rays.take(sweet_idxs, axis=0))

	wider_idxs = plane_filters.filter_bounding_planes(rays, mesh_and_bounds_msg.knowledge_bounding_planes[0].coef, mesh_and_bounds_msg.knowledge_bounding_planes[1].coef, mesh_and_bounds_msg.plane_sep_angle_gt_pi)
	
	partitioned_rays.append(rays.take([x for x in wider_idxs if x not in sweet_idxs], axis=0))

	print partitioned_rays
	print "sweet_shape: ", partitioned_rays[0].shape, " wider_shape: ", partitioned_rays[1].shape
	raw_input("How does that partition look?")
	return partitioned_rays

def remove_prev_object():
	if obj_name != '':
		print "Removing object: ", obj_name
		remove = env.GetKinBody(obj_name)
		env.RemoveKinBody(remove)
	else:
		print "No object in simulation environment to remove."


def load_new_object(mesh_path):
	#newObject = raw_input("Enter filename (include tag) for loading: ")
	env.Load(mesh_path)
	no_extension = mesh_path.split('.')
	path_array = no_extension[0].split('/')
	obj_name = path_array[-1]

	return obj_name

#The TransformToPose function will take the trajectory that openrave finds and
#transform them into orientation and position, for setting up a pose that can be
#sent to MoveIt! storing in a message list
def TransformToPose( G ):
	"""
	Compute geometry_msgs::Pose from 4x4 transform
	G is a 4x4 matrix which is [R t; 0 0 0 1]
	"""
	tj = array( G )[0:3,3]
	q = tr.quaternion_from_matrix( G )
	orientation = geometry_msgs.msg.Quaternion( q[0], q[1], q[2], q[3] )
	position = geometry_msgs.msg.Point( tj[0], tj[1], tj[2] )
	pose = geometry_msgs.msg.Pose( position, orientation )
	return pose

def listen_for_meshes():
	rospy.Subscriber("convex_hull/openrave_params", Mesh_and_bounds, full_info_callback)
	rospy.spin()

def full_info_callback(msg):
	print "Got a Mesh_and_bounds_msg!"
	print "Plane1: ", msg.ninety_degree_bounding_planes[0]
	print "Plane2: ", msg.ninety_degree_bounding_planes[1]
	
	replace_target(msg.convex_hull)
	
	main(msg)

def replace_target(convex_hull):
	new_mesh = TriMesh()
	new_mesh.vertices = []
	for vertex in convex_hull.vertices:
		new_mesh.vertices.append([vertex.x, vertex.y, vertex.z])
	#print "convex_hull indices: ", convex_hull.triangles
	new_mesh.indices = []
	for triangle_mesh in convex_hull.triangles:
		new_mesh.indices.append(list(triangle_mesh.vertex_indices))

	#print new_mesh.indices
	#print new_mesh.vertices
	#print dir(new_mesh)
	grasp_target = env.GetKinBody('grasp_target')
	env.RemoveKinBody(grasp_target)
	grasp_target.InitFromTrimesh(new_mesh, True)
	env.AddKinBody(grasp_target)

	#lol = raw_input("Pausing...")

def listen_for_LR_hand():
	return rospy.Subscriber("grasping_hand_selection", String, set_hand_callback)

def set_hand_callback(msg):
	global cur_hand
	global loaded_hands
	global arm_type

	if msg.data == "L" or msg.data == "l":
		hand_type = os.environ.get("FLOR_LEFT_HAND_TYPE", "")
		arm_type = "L"

	elif msg.data == "R" or msg.data == "r":
		hand_type = os.environ.get("FLOR_RIGHT_HAND_TYPE", "")
		arm_type = "R"
	else:
		print "Unsupported arm selection in OpenRAVE/set_hand_callback(): ", msg.data

	if hand_type in loaded_hands:
		print "New hand in use: ", hand_type
		cur_hand = hand_type
	else:
		print "Unsupported hand: ", hand_type, " reusing current hand: ", cur_hand
		print "Please add ", hand_type, " to the loaded_hands dictionary in SimEnvLoading.py"
	

def publish_poses(pose_array, mesh_ref_frame):
	pub = rospy.Publisher('adept_wrist_orientation', PoseArray, queue_size=10)
	pose_array = change_pose_ref_frame(pose_array, mesh_ref_frame)
	pose_msg = PoseArray()
	pose_msg.poses = pose_array
	pose_msg.header.stamp = rospy.Time.now()
	pose_msg.header.frame_id = mesh_ref_frame
	
	pub.publish(pose_msg)

def change_pose_ref_frame(pose_array, mesh_ref_frame):
	print "Need to test if transforming poses worked properly"
	global final_pose_frame
	if final_pose_frame != mesh_ref_frame:
		for idx, pose in enumerate(pose_array):
			pose.header.frame_id = mesh_ref_frame
			pose_array[idx] = tr.transformPose(final_pose_frame, pose)

	return pose_array

#def get_pelvis_to_x_transform(mesh_ref_frame):
#	global fixed_ref_frame
#	global tf_listener
#	while not rospy.is_shutdown():
#		try:
#			(trans, rot) = tf_listener.lookupTransform("/" + mesh_ref_frame, "/" + fixed_ref_frame, rospy.Time(0))
#			break
#		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#			continue
#
#	return trans, rot

if __name__ == '__main__':
	rospy.init_node('SimEnvLoading', anonymous=True)
	global pelvis_listener
	pelvis_listener = tf.TransformListener()
	set_openrave_environment_vars()
	query_final_pose_frame()
	build_environment()
	#main('lol')
	hand_subscriber = listen_for_LR_hand()
	listen_for_meshes()
