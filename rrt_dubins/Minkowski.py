def minkowski_sum_circle(robot_radius,obstacle_radius):
	obstacle_radius+=2*robot_radius
	return obstacle_radius


def minkowski_sum_square(robot_radius,obstacle_width):
	obstacle_width+=4*robot_radius
	return obstacle_width