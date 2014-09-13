import random
from numpy import array
import math

def generate_grasp_params(gmodel, mesh_and_bounds_msg):
	params = get_params(gmodel)

	filtered_ray_idxs = filter_approach_rays(params['approachrays'], mesh_and_bounds_msg)
	#params['rolls'] = limit_wrist_rolling()
	#params['approachrays'] = gmodel.computeBoxApproachRays()
	#print dir(params['approachrays'])
	#print "__doc__: ", params['approachrays'].__doc__
	#print "__module__: ", params['approachrays'].__mod__#, params['approachrays'].__module__
	#print "base: ", params['approachrays'].bases
	#print params['approachrays']
	print filtered_ray_idxs
	
	#print "standoffs: ", params['standoffs']
	#print "rolls: ", params['rolls']
	params['approachrays'] = params['approachrays'].take(filtered_ray_idxs, axis=0)
	print params['approachrays']
	print params['graspingnoise']
	#params['graspingnoise'] = (0, 0)
	#params['normalanglerange'] = 0
	#params['spheredelta'] = 0
	params['standoffs'] = array([0])
	#params['directiondelta'] = 0
	#gmodel.numthreads = 3

	#gmodel.generate(**params)
	return params

def get_params(gmodel):
	preshapes,standoffs,rolls,approachrays, graspingnoise,forceclosure,forceclosurethreshold,checkgraspfn,manipulatordirections,translationstepmult,finestep,friction,avoidlinks,plannername = gmodel.autogenerateparams()

	all_locals = locals()
	all_params = {}
	for key, value in all_locals.iteritems():
		if key != 'gmodel':
			all_params[key] = value

	return all_params

def filter_approach_rays(initial_approachrays, bounding_plane_msg):
	plane1 = bounding_plane_msg.bounding_planes[0].coef
	plane2 = bounding_plane_msg.bounding_planes[1].coef
	planes_are_obtuse = bounding_plane_msg.plane_sep_angle_gt_pi
	cur_ray_idxs = filter_bounding_planes(initial_approachrays, plane1, plane2, planes_are_obtuse)

	num_return_rays = 10
	cur_ray_idxs = random_ray_selection(cur_ray_idxs, num_return_rays)

	return cur_ray_idxs

def filter_bounding_planes(initial_approach_rays, bplane1, bplane2, planes_are_obtuse):
	out_ray_idxs = []
	for idx, ray in enumerate(initial_approach_rays):
		pt_to_bplane1_dist = get_plane_dist(ray[0:3], bplane1)
		pt_to_bplane2_dist = get_plane_dist(ray[0:3], bplane2)

		if pt_to_bplane1_dist >= 0 and pt_to_bplane2_dist >= 0:
			print ray, " inside both bounding planes."
			out_ray_idxs.append(idx)

		elif planes_are_obtuse and (pt_to_bplane1_dist >= 0 or pt_to_bplane2_dist >= 0):
			print ray, " inside one of the obtuse planes."
			out_ray_idxs.append(idx)
		else:
			print ray, " is not inside the bounded region. dist1: ", pt_to_bplane1_dist, " dist2: ", pt_to_bplane2_dist

	print "Initial ray count: ", len(initial_approach_rays), " final count: ", len(out_ray_idxs)
	return out_ray_idxs

def random_ray_selection(ray_idxs, num_return_rays):
	return random.sample(ray_idxs, num_return_rays)

# The normal of the planes should point TOWARD THE ROBOT
#	A positive plane distance is included in the result rays
def filter_plane(rays, plane_coefficient_list):
	out_rays = []
	for ray in rays:
		#print ray, plane_coefficient_list
		pt_to_plane_dist = get_plane_dist(ray[0:3], plane_coefficient_list)
		print "For point ", ray[0:3], " the point to plane distance is ", pt_to_plane_dist
		if pt_to_plane_dist >= 0:
			print "\tPoint is valid."
			out_rays.append(ray)

		else:
			print "\tPoint is invalid."

	return out_rays

def get_plane_dist(pt, plane_coefficient_list):
	dot = 0
	norm_len = 0;
	for idx in range(3):
		#print "Is it a list?: ", pt[idx], " ", plane_coefficient_list[idx]
		dot += (pt[idx] * plane_coefficient_list[idx])
		norm_len += plane_coefficient_list[idx] ** 2

	norm_len = norm_len ** 0.5
	dist = (dot + plane_coefficient_list[3]) / norm_len
	return dist


def limit_wrist_rolling():
	wrist_orientations = []
	cur_angle = -math.pi / 2
	angle_step = math.pi / 4
	for i in range(0, 5):
		wrist_orientations.append(cur_angle)
		cur_angle += angle_step

	return wrist_orientations
