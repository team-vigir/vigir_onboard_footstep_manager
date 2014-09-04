import plane_filters


def test_plane_dist():
	rays = []
	rays.append([0, 0, 0])
	rays.append([0, 0, 1])
	rays.append([1, 0, 1])
	rays.append([1, 1, 1])
	rays.append([20, 5, -1])

	plane_coefficients = [0, 0, 1, 0]
	plane_coefficients2 = [1, 1, 1, 0]
	plane_coefficients3 = [1, 0, 1, 1]

	res = plane_filters.filter_plane(rays, plane_coefficients)
	print "res: ", res , "\n\n\n"

	res2 = plane_filters.filter_plane(rays, plane_coefficients2)
	print "res2: ", res2 , "\n\n\n"

	res3 = plane_filters.filter_plane(rays, plane_coefficients3)
	print "res3: ", res3 , "\n\n\n"



if __name__ == '__main__':
	print "Initiating plane filters test."

	test_plane_dist()