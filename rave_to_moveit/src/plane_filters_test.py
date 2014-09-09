import plane_filters


def test_plane_dist():
	rays = mk_rays()

	plane_coefficients = [0, 0, 1, 0]
	plane_coefficients2 = [1, 1, 1, 0]
	plane_coefficients3 = [1, 0, 1, 1]

	res = plane_filters.filter_plane(rays, plane_coefficients)
	print "res: ", res , "\n\n\n"

	res2 = plane_filters.filter_plane(rays, plane_coefficients2)
	print "res2: ", res2 , "\n\n\n"

	res3 = plane_filters.filter_plane(rays, plane_coefficients3)
	print "res3: ", res3 , "\n\n\n"


def test_bounding_plane_filter():
	rays = mk_rays()
	rays.append([0, 1, 0])
	rays.append([0, 1, 1])
	rays.append([1.01, 1, 0])
	rays.append([1.5, 2, 1])

	plane_coefficients = [0, 0, 1, 0]
	plane_coefficients2 = [-1, 0, 0, 0]
	planes_are_obtuse = False

	expected_res = [[0,0,0], [0,0,1], [1,0,1], [1,1,1], [0,1,0], [0,1,1]]
	actual_res = filter_bounding_planes(rays, plane_coefficients, plane_coefficients2, planes_are_obtuse)

	for pt in expected_res:
		idx = actual_res.find(pt)
		if idx != -1:
			print "Found ", pt, " in bounding filter results as expected"
			del expected_res[idx]
		else:
			print "Expected ", pt " in bounding filter results, but NOT FOUND. FAIL."
	
	if expected_res.len() != 0:
		print "Improper filtering: Results are missing points - ", expected_res
			


def mk_rays():
	rays = []
	rays.append([0, 0, 0])
	rays.append([0, 0, 1])
	rays.append([1, 0, 1])
	rays.append([1, 1, 1])
	rays.append([20, 5, -1])

	return rays
	

if __name__ == '__main__':
	print "Initiating plane filters test."

	test_plane_dist()
