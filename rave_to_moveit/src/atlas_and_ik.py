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
		print "Initial Atlas Load successful."
	else:
		print "Atlas is None in load_atlas(). Load failed."
		exit()

	world_orientation = array([[1, 0, 0, 0], [0, 1, 0, 0,], [0, 0, 1, 0], [0, 0, 0, 1]])
	atlas.SetTransform(world_orientation)
	
	#set_atlas_manipulators(atlas)
	#load_ik(atlas)

	return atlas

def set_atlas_manipulators(atlas):
	base_manip_info = atlas.ManipulatorInfo()
	l_arm = base_manip_info
	l_arm._sBaseLinkName = 'utorso'
	l_arm._sEffectorLinkName = 'left_palm'
	l_arm._name = 'left_arm'
	l_arm._vClosingDirection = array([1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1])
	l_arm._vGripperJointNames = ['left_f0_j1', 'left_f0_j2', 'left_f0_j3', 'left_f1_j0', 'left_f1_j1', 'left_f1_j2', 'left_f1_j3', 'left_f2_j0', 'left_f2_j1', 'left_f2_j2', 'left_f2_j3']
	l_arm._vdirection = [0, 1, 0]

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


	#chaintree = solver.generateIkSolver(**req_info)
	#code = solver.writeIkSolver(chaintree)
	#open('ik.cpp', 'w').write(code)


def load_ik(atlas):
	global ikmodel
	ikmodel = databases.inversekinematics.InverseKinematicsModel(atlas,iktype=IkParameterizationType.Transform6D)
	if not ikmodel.load():
		print "Generating ikfiles using .autogenerate()"
		ikmodel.autogenerate()

def solve_ik(transform):
	global ikmodel
	return ikmodel.manip.FindIKSolutions(transform, IkFilterOptions.IgnoreEndEffectorEnvCollisions)


#NO STANDOFF SUPPORT
def filter_approach_rays_using_ik(approach_rays, rolls):
	global ikmodel

	out_rays = []
	for ray in approach_rays:
		for roll in rolls:
			transform = get_tranform_for_approach(ray[0:3], -ray[3:6], roll)
			solutions = ikmodel.manip.FindIKSolutions(transform)
			print "Solutions: ", solutions		
			if solutions is not None:
				print "Adding approach vector."
				out_rays.append(ray)

	return out_rays

def goal_pose_is_reachable(pos, approach, roll):
	
	return True

	transform = get_transform_for_approach(pos, approach, roll)
	solutions = solve_ik(transform)

	#print "Solutions: ", solutions
	#raw_input("Were there solutions?")
	if len(solutions) > 0:
		print "Solutions: ", solutions
		return True
	else:
		print "Grasp is not reachable. Skipping."
		return False

def get_transform_for_approach(pos, approach, roll):
	approach_dir_vec = approach
	palm_direction = [0, 1, 0]
	world_quat = [1, 0, 0, 0]
	palm_center_to_face_dist = 0.03 #meters

	quat_orient_approach = quatRotateDirection(palm_direction, approach_dir_vec)
	roll_quat = quatFromAxisAngle(approach_dir_vec, roll)
	
	pos = offset_from_center_of_palm(pos, approach_dir_vec, palm_center_to_face_dist)

	final_quat = quatMult(roll_quat, quatMult(world_quat, quat_orient_approach))
	final_pose = [final_quat[0], final_quat[1], final_quat[2], final_quat[3], pos[0], pos[1], pos[2]]
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
		a1 = gt.drawTransform(get_transform_for_approach(ray[0:3], -ray[3:6], math.pi/2))
		raw_input("How's the transform?")

def visualize_ik_solution(env, transform):
	solutions = solve_ik(transform)

	if len(solutions) > 0:
		print "Picking first solution"
		print "All solutions: ", solutions
		newrobot = RaveCreateRobot(env, ikmodel.robot.GetXMLId())
		newrobot.Clone(ikmodel.robot, 0)
		for link in newrobot.GetLinks():
			for geom in link.GetGeometries():
				geom.SetTransparency(0.8)
		env.Add(newrobot, True)
		newrobot.SetTransform(ikmodel.robot.GetTransform())
		indices = ikmodel.manip.GetArmIndices()
		newrobot.SetDOFValues(solutions[0], indices)
		joint_values = newrobot.GetDOFValues(indices)
		
		print "Arm joint values: " 
		for idx in range(0, len(joint_values)):
			print ikmodel.robot.GetJointFromDOFIndex(indices[idx]).GetName(), joint_values[idx]

	else:
		print "No IK solution found for transform: ", transform


