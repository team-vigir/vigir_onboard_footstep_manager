#!/usr/bin/env python
import rospy
from openravepy import *
import rospkg
import os
import numpy, time
from openravepy import ikfast
from openravepy.examples import fastgraspingthreaded
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
	#if os.environ.get("OPENRAVE_DATA", "") != "":
	#	os.environ["OPENRAVE_DATA"] = rave_to_moveit_path + os.environ["OPENRAVE_DATA"]
	#else:
	os.environ["OPENRAVE_DATA"] = rave_to_moveit_path

	#print "set env vars"

def build_environment():
	global env

	env = Environment()
	#load_hands()
	#env.Load('scenes/grasp_target.env.xml')
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

def main(mesh_and_bounds_msg): 
	#If there was a previous object, delete it:
	global env
	global obj_name
	global grasp_target

	print "robots: ", env.GetRobots()
	robot = env.GetRobots()[0]

	#robot.SetActiveDOFs(range(7, 16))
	#robot = env.GetRobot(loaded_hands[cur_hand][ENV_NAME])
	manipulator = robot.GetManipulators()[0]
	#robot.SetActiveDOFs(manipulator.GetGripperIndices(),DOFAffine.X+DOFAffine.Y+DOFAffine.Z)
	#print "direction: ", manipulator.GetDirection()
	#print "closing direction: ", manipulator.GetChuckingDirection()
	#print "palm direction: ", manipulator.GetPalmDirection()
	#manipulator.SetLocalToolDirection( [0, 0, 1])
	#manipulator.SetChuckingDirection([1, 1, 1, 1, 1, 1, 1, 1, 1])
	ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterizationType.Transform6D)
	if not ikmodel.load():
		ikmodel.autogenerate()
	target = env.GetKinBody(grasp_target)
	print "Got target!"

	gmodel = databases.grasping.GraspingModel(robot,target)
	if not gmodel.load():
		fgmodel =  fastgraspingthreaded.FastGraspingThreaded(robot, target)
		params = plane_filters.generate_grasp_params(gmodel, mesh_and_bounds_msg)
		#params = set_params_for_threaded_call(params, target)

		#num_grasps, grasps = fgmodel.callGraspThreaded(**params)
		gmodel.generate(**params)
	
	#gmodel.grasps = grasps
	show_grasps(gmodel)
	
	#print "grasps: ", gmodel.grasps
	graspnum = input("Please enter number of valid grasps to return: ")
	validgrasps, validindices = gmodel.computeValidGrasps(checkcollision=False, returnnum=graspnum)
	#print "validgrasps: ", validgrasps
	#print "validindices: ", validindices
	basemanip = interfaces.BaseManipulation(robot)
	#print dir(gmodel)
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
			traj = basemanip.MoveToHandPosition(matrices=[T],execute=False,outputtrajobj=True)
			print('Finished Grasp')
			robot.GetController().SetPath(traj)

			x += 1
		
		print pose_array

	publish_poses(pose_array, mesh_and_bounds_msg.header.frame_id)
	#raveLogInfo((T))

def show_grasps(gmodel):
	for grasp in gmodel.grasps:
		gmodel.showgrasp(grasp)
		#lol = raw_input("Pausing to observe grasp... Enter any key to continue")

def set_params_for_threaded_call(params, target):
	params['numthreads'] = 8 
	params['maxgrasps'] = 20
	del(params['plannername'])
	del(params['finestep'])
	del(params['forceclosure'])
	del(params['checkgraspfn'])
	params['forceclosurethreshold'] = 0.001
	params['graspingnoise'] = 0
	params['ngraspingnoiseretries'] = 0
	params['checkik'] = False #FOR NOW!
	params['target'] = target

	return params


def mk_fake_openrave_params_msg():
	msg = Mesh_and_bounds()
	msg.full_abs_mesh_path = ""
	msg.convex_hull = None
	msg.bounding_planes[0].coef = [1, 0, 0, 0]
	msg.bounding_planes[1].coef = [-1, 0, 0, 0]
	msg.plane_sep_angle_gt_pi = True

	return msg

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
	msg = mk_fake_openrave_params_msg()
	main(msg)
	#hand_subscriber = listen_for_LR_hand()
	#listen_for_meshes()
