#!/usr/bin/env python
import rospy
from openravepy import *
import rospkg
import os
import numpy, time
from openravepy import ikfast
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray
from hullify.msg import Mesh_and_bounds

import geometry_msgs
from tf import transformations as tr
import tf_conversions.posemath as pm
from numpy import pi, eye, dot, cross, linalg, sqrt, ceil, size
from numpy import hstack, vstack, mat, array, arange, fabs, zeros
import math

import plane_filters

env = None
obj_name = ''

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
	env = Environment()
	env.Load('scenes/test2.env.xml')
	env.SetViewer('qtcoin')

#The main function grabs the environment you want, setting the robot as the first
#robot tag put into your env file. Then it sets up the ikfast, the target for the robot
#and the database grasps are store in. Then send to the TransformToPose function
def main(mesh_and_bounds_msg): 
	#If there was a previous object, delete it:
	global env
	global obj_name
	if obj_name != '':
		print "Removing object: ", obj_name
		remove = env.GetKinBody(obj_name)
		env.RemoveKinBody(remove)
	else:
		print "No object in simulation environment to remove."

	newObject = mesh_and_bounds_msg.full_abs_mesh_path
	#newObject = raw_input("Enter filename (include tag) for loading: ")
	env.Load(newObject)
	comp = newObject.split('.')
	path = comp[0].split('/')
	obj_name = path[-1]
	print "obj_name (grasp_target): ", obj_name

	#REMOVE CODE!
	#remove = env.GetKinBody(comp[0])
	#env.RemoveKinBody(remove)

	#grasp_target = raw_input("Please enter name of object you want to grasp: ")
	grasp_target = obj_name

	robot = env.GetRobots()[0]
	ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterizationType.Transform6D)
	if not ikmodel.load():
		ikmodel.autogenerate()
	target = env.GetKinBody(grasp_target)
	print "Got target!"

	aabb=env.GetRobots()[0].ComputeAABB()
	#env.assertGreaterEqual(aabb.pos()[-1]-aabb.extents()[-1],0)
	if aabb.pos()[-1]-aabb.extents()[-1] < 0:
		print "Bounding box assertion failed in main()"
		print aabb.pos()[-1]
		print aabb.extents()[-1]
		exit()

	gmodel = databases.grasping.GraspingModel(robot,target)

	if not gmodel.load():
		plane_filters.generate_potential_grasps(gmodel, mesh_and_bounds_msg)
	
	#generate_potential_grasps(gmodel)

	graspnum = input("Please enter number of valid grasps to return: ")
	validgrasps, validindicees = gmodel.computeValidGrasps(returnnum=graspnum)
	basemanip = interfaces.BaseManipulation(robot)
	print dir(gmodel)
	pose_array = []
	with robot:
		for x in range (0,graspnum):
			grasp = validgrasps[x]
			gmodel.setPreshape(grasp)
			T = gmodel.getGlobalGraspTransform(grasp,collisionfree=True)
			p = TransformToPose(T)
			pose_array.append(p)
			traj = basemanip.MoveToHandPosition(matrices=[T],execute=False,outputtrajobj=True)
			print('Finished Grasp')
			robot.GetController().SetPath(traj)
			#robot.WaitForController(0)
		
		print pose_array

	publish_poses(pose_array, mesh_and_bounds_msg.header.frame_id)
	#raveLogInfo((T))

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
	print "Plane1: ", msg.bounding_planes[0]
	print "Plane2: ", msg.bounding_planes[1]
	main(msg)

def publish_poses(pose_array, mesh_ref_frame):
	pub = rospy.Publisher('adept_wrist_orientation', PoseArray, queue_size=10)
	pose_msg = PoseArray()
	pose_msg.poses = pose_array
	pose_msg.header.stamp = rospy.Time.now()
	pose_msg.header.frame_id = mesh_ref_frame
	
	pub.publish(pose_msg)

if __name__ == '__main__':
	rospy.init_node('SimEnvLoading', anonymous=True)
	set_openrave_environment_vars()
	build_environment()
	#main('lol')
	listen_for_meshes()
