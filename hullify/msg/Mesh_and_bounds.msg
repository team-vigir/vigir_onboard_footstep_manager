Header header
string full_abs_mesh_path #Soon to be DEPRECATED
shape_msgs/Mesh convex_hull
shape_msgs/Plane[2] ninety_degree_bounding_planes	#The normals point toward the robot
shape_msgs/Plane[2] knowledge_bounding_planes	#Normals also point toward robot
bool plane_sep_angle_gt_pi	#This applies to the less_stringent planes, the stringent ones will always be less than PI/2 off
