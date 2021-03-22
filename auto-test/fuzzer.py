import rospy
from pb_msgs.msg import PerceptionObstacles
import numpy as np
import random

def talker():
	pub = rospy.Publisher('/apollo/perception/obstacles', PerceptionObstacles, queue_size = 10)

	rospy.init_node('talker', anonymous = True)
	# define the frequency of refresh (Hz)
	rate = rospy.Rate(0.2)
	 
	# define the coordinates of the start and end points
	# can be optimised to read from files
	start_x = 587701
	start_y = 4141470
	end_x = 587065
	end_y = 4141583

	# define the boundary of region where obstacles are generated
	# using the start and end points in this case
	bound_left = end_x
	bound_right = start_x
	bound_up = end_y
	bound_down = start_y
	
	# calculate the area of the region
	area_region = (bound_right - bound_left) * (bound_up - bound_down)
	# define the obstacle density (0 - 1)
	obstacle_density = 0.005

	# define number of obstacles
	n_obstacles = int(obstacle_density * area_region)

	
	while not rospy.is_shutdown():
		# create PerceptionObstacles object
		msg_obstacles = PerceptionObstacles()

		for obs in range(n_obstacles):
			msg = msg_obstacles.perception_obstacle.add()
			msg.id = obs
			# randomly assign x and y coordinates to the obstacle
			x = random.uniform(bound_left, bound_right)
			y = random.uniform(bound_down, bound_up)
			msg.position.x = x
			msg.position.y = y
			msg.position.z = 0

			# assign random theta to the obstacle
			# theta represents the heading direction of the obstacle
			# msg.theta = random.uniform(1, 2)
			msg.theta = 2.0
		
			# custom obstacle dimentions
			msg.length = 2.0
			msg.width = 2.0
			msg.height = 2.0

			# define the type of the obstacle (default 10: general obstacle)
			msg.type = random.randrange(0, 5)	

			# define the velocity of the obstacle
			# msg.velocity.x = 5
			# msg.velocity.y = 5

		# publish the obstacles to ROS topic '/apollo/perception/obstacles'
		pub.publish(msg_obstacles)
		rate.sleep()
		

if __name__ == '__main__':
	try:
		talker()
		
	except rospy.ROSInterruptException:
		pass
