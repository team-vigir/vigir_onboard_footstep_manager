#!/usr/bin/env python
import rospy
from openravepy import *
from openravepy.examples import tutorial_grasptransform
import rospkg
import os
import numpy, time
from std_msgs.msg import String

from numpy import pi, eye, dot, cross, linalg, sqrt, ceil, size
from numpy import hstack, vstack, mat, array, arange, fabs, zeros
import math

import grasping
import plane_filters
import atlas_and_ik
import openraveIO

gt = None
cur_hand = "l_robotiq"
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
	global gt	
	env = Environment()
	#load_hands()
	robot = atlas_and_ik.load_atlas(env)
	env.Load('scenes/grasp_target.env.xml')
	target = env.GetKinBody('grasp_target')
	#env.Load('adeptsetup.robot.xml')

	env.SetViewer('qtcoin')
	gt = tutorial_grasptransform.GraspTransform(env,target)
	view_robot_ref_frames(robot)

	return env, robot, target

def view_robot_ref_frames(robot):
	global gt
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
	while True:
		suggestion = raw_input("Please input the reference frame of the final pose: \n\t0 - new \n\t1 - /world: ")
		if suggestion == "0":
			final_pose_frame = raw_input("what's your new frame? (please give it a slash prefix")
		elif suggestion == "1":
			final_pose_frame = "/world"

		else:
			continue

		break
	return final_pose_frame

class VigirGrasper:
	def __init__(self, env, robot, target):
		print "Making grasper"
		self.env = env
		self.robot = robot
		self.target = target
		self.gmodel = gmodel = grasping.GraspingModel(robot,target)
		self.totalgrasps = []
		self.raveio = None

	def set_io(self, io_obj):
		self.raveio = io_obj

	def replace_target(self, convex_hull):
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
		grasp_target = self.env.GetKinBody('grasp_target')
		self.env.RemoveKinBody(grasp_target)
		grasp_target.InitFromTrimesh(new_mesh, True)
		self.env.AddKinBody(grasp_target)

		#lol = raw_input("Pausing...")

	def find_grasps(self, mesh_and_bounds_msg): 
		global ikmodel
		global gt

		self.totalgrasps = []

		#gmodel = grasping.GraspingModel(robot,target)
		params = plane_filters.generate_grasp_params(self.gmodel, mesh_and_bounds_msg)
		self.totalgrasps = self.get_grasps(mesh_and_bounds_msg, params, gt, returnnum=15)
		
		if len(self.totalgrasps) == 0:
			print "No suitable grasps found. Please select another pointcloud."
			return

		print len(self.totalgrasps), " Grasps available."
		self.show_grasps(self.totalgrasps)
		graspnum = "lol"
		while int(graspnum) is false:
			graspnum = input("Please enter number of valid grasps to return: ")
		
		pose_array = []
		with robot:
			x = 0
			while (x < graspnum) and x < len(self.totalgrasps):
				grasp = self.totalgrasps[x]
				T = self.gmodel.getGlobalGraspTransform(grasp,collisionfree=True)
				p = raveio.TransformToPoseStamped(T)
				pose_array.append(p)

				x += 1
		
			print "pose_array: ", pose_array

		self.raveio.publish_poses(pose_array)
		self.show_ik_on_request()

	def get_grasps(self, mesh_and_bounds_msg, params, gt, returnnum=5):
		grasps = []
		partitioned_rays = partition_rays(mesh_and_bounds_msg, params['approachrays'])

		if sum([len(x) for x in partitioned_rays]) < 1:
			print "Insufficient approach rays generated. How do the planes look? Returning null pose."
			return []

		for rays in partitioned_rays:
			params['approachrays'] = rays
			params['remaininggrasps'] = returnnum - len(grasps)
			#atlas_and_ik.visualize_approaches(gt, params)
			
			self.gmodel.generate(**params)
			grasps.extend(self.gmodel.grasps)

			if len(grasps) >= returnnum:
				return grasps
				

		return grasps

	def show_grasps(self, grasps):
		for grasp in grasps:
			self.gmodel.showgrasp(grasp)

	def show_ik_on_request(self):
		res = raw_input("Input the index of the grasp you would like IK for (q to quit): ")
		while res != "q" or res != "Q":
			transform = self.gmodel.getGlobalGraspTransform(self.totalgrasp[res],collisionfree=True)
			atlas_and_ik.visualize_ik_solution(self.env, transform)
	
def partition_rays(mesh_and_bounds_msg, rays):
	partitioned_rays = []
	sweet_idxs = plane_filters.filter_bounding_planes(rays, mesh_and_bounds_msg.ninety_degree_bounding_planes[0].coef, mesh_and_bounds_msg.ninety_degree_bounding_planes[1].coef, False)

	partitioned_rays.append(rays.take(sweet_idxs, axis=0))

	wider_idxs = plane_filters.filter_bounding_planes(rays, mesh_and_bounds_msg.knowledge_bounding_planes[0].coef, mesh_and_bounds_msg.knowledge_bounding_planes[1].coef, mesh_and_bounds_msg.plane_sep_angle_gt_pi)
	
	partitioned_rays.append(rays.take([x for x in wider_idxs if x not in sweet_idxs], axis=0))

	#print partitioned_rays
	#print "sweet_shape: ", partitioned_rays[0].shape, " wider_shape: ", partitioned_rays[1].shape
	#raw_input("How does that partition look?")
	return partitioned_rays

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
	


if __name__ == '__main__':
	rospy.init_node('SimEnvLoading', anonymous=True)
	
	set_openrave_environment_vars()
	env, robot, target = build_environment()
	grasper = VigirGrasper(env, robot, target)
	final_pose_frame = query_final_pose_frame()
	mesh_ref_frame = "/pelvis"
	raveio = openraveIO.openraveIO(grasper, final_pose_frame, mesh_ref_frame)
	grasper.set_io(raveio)

	#openraveIO.testpublisher(raveio)

	hand_subscriber = listen_for_LR_hand()

	print "Awaiting hulls..."
	rospy.spin()
