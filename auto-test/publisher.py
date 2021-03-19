import rospy
from pb_msgs.msg import PerceptionObstacles
import numpy as np

def talker():
	pub = rospy.Publisher('/apollo/perception/obstacles', PerceptionObstacles, queue_size = 10)

	rospy.init_node('talker', anonymous = True)
	rate = rospy.Rate(10)
	 
	while not rospy.is_shutdown():
		# generate an simple obstacle
		msg1 = PerceptionObstacles()
		msg = msg1.perception_obstacle.add()
		msg.id = 1234567
		
		if (np.random.rand() > .5):
			msg.position.x = 587226
			msg.position.y = 4141535
		else:
			msg.position.x = 587210
			msg.position.y = 4141560

		
		msg.position.z = 0

		msg.theta = 1.70768520645
		
		msg.length = 10.0
		msg.width = 10.0
		msg.height = 10.0

		pub.publish(msg1)
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
		
	except rospy.ROSInterruptException:
		pass
