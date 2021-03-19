import rospy
from pb_msgs.msg import PerceptionObstacles
import numpy as np
import random

def talker():
	pub = rospy.Publisher('/apollo/perception/obstacles', PerceptionObstacles, queue_size = 10)

	rospy.init_node('talker', anonymous = True)
	# define the time of refresh
	rate = rospy.Rate(6000)
	 
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
	obstacle_density = 0.0005

	# define number of obstacles
	n_obstacles = int(obstacle_density * area_region)

	msg1 = PerceptionObstacles()
	
	while not rospy.is_shutdown():

		msg_obstacles = PerceptionObstacles()
		# generate various obstacles
		for obs in range(n_obstacles):
			msg = msg_obstacles.perception_obstacle.add()
			msg.id = obs
			# randomly assign x and y coordinates to the obstacle
			x = random.uniform(bound_left, bound_right)
			y = random.uniform(bound_down, bound_up)
			msg.position.x = x
			msg.position.y = y
			msg.position.z = 0

			msg.theta = random.uniform(1, 2)
		
			msg.length = 2.0
			msg.width = 2.0
			msg.height = 2.0

		pub.publish(msg_obstacles)
		rate.sleep()
		
		# # generate an simple obstacle
		# msg = PerceptionObstacles()
		# msg1 = msg.perception_obstacle.add()
		# msg1.id = 1234567

		# msg1.position.x = 587226
		# msg1.position.y = 4141535
		# msg1.position.z = 0

		# msg1.theta = 1.70768520645
		
		# msg1.length = 5.0
		# msg1.width = 5.0
		# msg1.height = 5.0

		# msg2 = msg.perception_obstacle.add()
		# msg2.id = 7654321

		# msg2.position.x = 587210
		# msg2.position.y = 4141560
		# msg2.position.z = 0

		# msg2.theta = 1.564424
		
		# msg2.length = 5.0
		# msg2.width = 5.0
		# msg2.height = 5.0



		# pub.publish(msg)
		# rate.sleep()

if __name__ == '__main__':
	try:
		talker()
		
	except rospy.ROSInterruptException:
		pass
