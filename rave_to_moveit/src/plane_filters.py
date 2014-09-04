##Jackson's Modifications
def generate_potential_grasps(gmodel, plane1, plane2):
	params = get_params(gmodel)

	params['approachrays'] = filter_approach_rays(params['approachrays'], plane1, plane2)

	gmodel.generate(**params)

def get_params(gmodel):
	preshapes,standoffs,rolls,approachrays, graspingnoise,forceclosure,forceclosurethreshold,checkgraspfn,manipulatordirections,translationstepmult,finestep,friction,avoidlinks,plannername = gmodel.autogenerateparams()

	all_locals = locals()
	all_params = {}
	for key, value in all_locals:
		if key != 'gmodel':
			all_params[key] = value

	print all_params, "\nExiting!"
	exit()

	return all_params

def filter_approach_rays(initial_approachrays, plane1, plane2):
	cur_rays = filter_plane(initial_approachrays, plane1)
	cur_rays = filter_plane(cur_rays, plane2)

	#cur_rays = random_ray_selection(cur_rays)

	return cur_rays

# The normal of the planes should point TOWARD THE ROBOT
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

def full_info_callback(msg):
	print "Got a Mesh_and_bounds_msg!"
	print "Plane1: ", msg.bounding_planes[0]
	main(msg.full_abs_mesh_path, msg.bounding_planes[0], msg.bounding_planes[1])

