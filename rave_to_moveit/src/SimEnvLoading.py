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
obj_name = ""
cur_hand = "l_robotiq"
grasp_target = 'grasp_target'
arm_type = "L"

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

	env = Environment()
	#load_hands()
	env.Load('scenes/test2.env.xml')
	
	env.SetViewer('qtcoin')

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

#The main function grabs the environment you want, setting the robot as the first
#robot tag put into your env file. Then it sets up the ikfast, the target for the robot
#and the database grasps are store in. Then send to the TransformToPose function
def main(mesh_and_bounds_msg): 
	#If there was a previous object, delete it:
	global env
	global obj_name
	global grasp_target

	#remove_prev_object()
	#obj_name = load_new_object(mesh_and_bounds_msg.full_abs_mesh_path)
	#print "obj_name (grasp_target): ", obj_name
	
	#grasp_target = raw_input("Please enter name of object you want to grasp: ")

	print "robots: ", env.GetRobots()
	robot = env.GetRobots()[0]
	#robot = env.GetRobot(loaded_hands[cur_hand][ENV_NAME])
	ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterizationType.Transform6D)
	if not ikmodel.load():
		ikmodel.autogenerate()
	target = env.GetKinBody(grasp_target)
	print "Got target!"

	gmodel = databases.grasping.GraspingModel(robot,target)
	#print dir(gmodel.grasper)
	#print gmodel.grasper.__class__
	#lol = input ("Pausing... Please input: ")
	if not gmodel.load():
		params = plane_filters.generate_grasp_params(gmodel, mesh_and_bounds_msg)
		gmodel.generate(**params)

	
	graspnum = input("Please enter number of valid grasps to return: ")
	validgrasps, validindices = gmodel.computeValidGrasps(returnnum=graspnum)
	print "validgrasps: ", validgrasps
	print "validindices: ", validindices
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
	print "Plane1: ", msg.bounding_planes[0]
	print "Plane2: ", msg.bounding_planes[1]
	
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
	grasp_target.InitFromTrimesh(new_mesh)
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
	hand_subscriber = listen_for_LR_hand()
	listen_for_meshes()
