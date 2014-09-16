from openravepy import *
from openravepy import ikfast
import openravepy

from numpy import pi, eye, dot, cross, linalg, sqrt, ceil, size
from numpy import hstack, vstack, mat, array, arange, fabs, zeros

import math

ikmodel = None

def load_atlas(env):
	#env.Load('robots/atlas/flor_atlas.dae')
	env.Load('robots/atlas/atlas_setup.robot.xml')
	atlas = env.GetRobot('atlas')
	
	if atlas is not None:
		print "Load successful."
	else:
		print "Atlas is None in load_atlas(). Load failed."
		exit()

	set_atlas_manipulators(atlas)

	return atlas

def set_atlas_manipulators(atlas):
	base_manip_info = atlas.ManipulatorInfo()
	l_arm = base_manip_info
	l_arm._sBaseLinkName = 'utorso'
	l_arm._sEffectorLinkName = 'l_hand'#'left_palm'
	l_arm._name = 'left_arm'
	l_arm._vClosingDirection = array([1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1])
	l_arm._vGripperJointNames = ['left_f0_j1', 'left_f0_j2', 'left_f0_j3', 'left_f1_j0', 'left_f1_j1', 'left_f1_j2', 'left_f1_j3', 'left_f2_j0', 'left_f2_j1', 'left_f2_j2', 'left_f2_j3']
	l_arm._vdirection = [0, 0, 1]

	atlas.AddManipulator(l_arm)
	l_arm_ptr = atlas.GetManipulators()[0]
	atlas.SetActiveManipulator(l_arm_ptr)
	print dir(l_arm_ptr)
	print atlas.GetManipulators()
	#lol = raw_input("pausing, did we get a manipulator?")
	load_arm_ik(atlas, l_arm_ptr)

def load_arm_ik(atlas, manip):
	#print "Need to autogenerate atlas IK"
	solver = ikfast.IKFastSolver(kinbody=atlas)
	links = atlas.GetLinks()
	req_info = {}
	req_info['baselink'] = manip.GetBase().GetIndex()
	req_info['eelink'] = manip.GetEndEffector().GetIndex()
	req_info['freeindices'] = []
	req_info['solvefn'] = ikfast.IKFastSolver.solveFullIK_6D

	print "Base link: ", req_info['baselink']
	print "eelink: ", req_info['eelink']

	print "Generating ikfiles"
	#chaintree = solver.generateIkSolver(**req_info)
	#code = solver.writeIkSolver(chaintree)
	#open('ik.cpp', 'w').write(code)


def load_ik():
	global ikmodel
	global robot
	ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterizationType.Transform6D)
	if not ikmodel.load():
		ikmodel.autogenerate()

#NO STANDOFF SUPPORT
# Could perhaps be better to integrate the filtering with the grasp check...
#	so, send in roll, position and approach dir and let that filter.
def filter_approach_rays_using_ik(approach_rays, rolls):
	global ikmodel

	out_rays = []
	for ray in approach_rays:
		transform = get_tranform_for_approach(ray, rolls)
		solutions = ikmodel.manip.FindIKSolutions(transform)
		print "Solutions: ", solutions		
		if solutions is not None:
			print "Adding approach vector."
			out_rays.append(ray)

	return out_rays

def get_transform_for_approach(ray, rolls):
	approach_dir_vec = -ray[3:6]
	palm_direction = [0, 1, 0]
	world_quat = [1, 0, 0, 0]
	roll_quats = []
	palm_center_to_face_dist = 0.03 #meters

	quat_orient_approach = quatRotateDirection(palm_direction, approach_dir_vec)
	for roll in rolls:
		roll_quats.append(quatFromAxisAngle(approach_dir_vec, roll))
	
	ray[0:3] = offset_from_center_of_palm(ray[0:3], approach_dir_vec, palm_center_to_face_dist)

	final_quat = quatMult(roll_quats[0], quatMult(world_quat, quat_orient_approach))
	final_pose = [final_quat[0], final_quat[1], final_quat[2], final_quat[3], ray[0], ray[1], ray[2]]
	print "final_pose: ", final_pose	
	transform = matrixFromPose(final_pose)

	return transform

def offset_from_center_of_palm(initial_pos, approach_dir, dist):
	mult_fact = dist * ((approach_dir[0]**2 + approach_dir[1]**2 + approach_dir[2]**2) ** 0.5)
	offset = [a * mult_fact for a in approach_dir]
	return [p - q for p, q in zip(initial_pos, offset)]

def test_transforms(gt):
	rays = [[1, 0, 0, 0, 0, 1],
		[1, 0, 0, 1, 0, 0],
		[0.3, 0.2, 1, 0, 1, 1]]
	rolls = [math.pi / 2]

	for ray in rays:
		transform = get_transform_for_approach(ray, rolls)
		a1 = gt.drawTransform(transform)
		raw_input("Drew new transform frame. Pausing before leaving scope...")

def visualize_approaches(gt, params):
	for ray in params['approachrays']:		
		a1 = gt.drawTransform(get_transform_for_approach(ray, [math.pi/2]))
		raw_input("How's the transform?")
