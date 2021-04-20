import numpy as np
from numpy import genfromtxt
import os, sys, time
from routing_pub import talker as rtalker
import rospy
from pb_msgs.msg import PerceptionObstacles

start = time.time()

# load data from data/obstacles.csv and data/collision.csv
obstacles_data = genfromtxt('/apollo/auto-test/data/obstacles.csv', delimiter=',')

# check whether collision.csv exists, it should not exist if no collision is detected
if (not os.path.exists('/apollo/auto-test/data/collision.csv')):
    print('No collision detected in the simulation!')
    sys.exit()

collision_data = genfromtxt('/apollo/auto-test/data/collision.csv', delimiter=',')

# remove duplicated data in collision_data
collision_data = np.unique(collision_data)
collision_size = collision_data.size
# find the information of all the collision obstacles in obstacle_data
collision_obstacle_data = np.zeros([collision_size, 10])
for c in range(collision_size):
    obstacle_info = obstacles_data[obstacles_data[:,0] == collision_data[c]]
    collision_obstacle_data[c] = obstacle_info[0]



# print collision rate
# get the number of scenarios generated
scenario_number = int(obstacles_data[-1][1])
general_rate = float(collision_size) / float(scenario_number)
obstacle_number = int(obstacles_data.shape[0])
# check whether Obstacle number is divisable by scenario number
if (obstacle_number % scenario_number != 0):
    print("Error: Obstacle number is not divisable by scenario number! Something goes wrong!")
    sys.exit()

obstacle_per_scenario = obstacle_number / scenario_number
print("Obstacle per scenario: %d" % obstacle_per_scenario)
print("Total Obstacle Generated: %d" % obstacle_number)
print("Scenario Number: %d" % scenario_number)
print("Number of Collision Detected: %d" % collision_size)
print("General Collision Rate (#collision / #scenerio): %.4f%%" % (general_rate * 100))

collision_analysis = np.array([[obstacles_data.shape[0], scenario_number, collision_size, general_rate]])
with open('/apollo/auto-test/data/collision_analysis.csv', 'a') as csv:
    np.savetxt(csv, collision_analysis, fmt=['%d','%d','%d','%.6f'], delimiter=',')

# regenerate the follow-up scenarios based on the obstacle data and MR
# MR: Suppose that in a driving scenario,S, a car collided with a static obstacle O at location L, 
#     Construct a follow-up driving scenario,S', that is identical to S except that O is repositioned 
#     slightly further away from the previous position. Run the follow-up driving scenario. The car 
#     should not stop before L.

# generate the obstacles based on obstacle.csv
pub = rospy.Publisher('/apollo/perception/obstacles', PerceptionObstacles, queue_size=10)
rospy.init_node('talker', anonymous=True)
# define the frequency of refresh (Hz)
rate = rospy.Rate(0.2)

# executive external script routing_pub.py to send routing request
# rtalker()
print("Fuzzer2 start to before obs gen: ", time.time() - start)

# generate the same number of scenarios 
for scenario in range(1, scenario_number+1):
    # create PerceptionObstacles object
    msg_obstacles = PerceptionObstacles()
    # obtain the data for the scenario of the loop
    obs_info = obstacles_data[obstacles_data[:,1] == scenario]
    for obs in range(obstacle_per_scenario):
        obs_info_one = obs_info[obs]
        msg = msg_obstacles.perception_obstacle.add()
        msg.id = int(obs_info_one[0])
        msg.position.x = obs_info_one[2]
        msg.position.y = obs_info_one[3]
        msg.position.z = obs_info_one[4]
        msg.theta = obs_info_one[5]
        msg.length = obs_info_one[6]
        msg.width = obs_info_one[7]
        msg.height = obs_info_one[8]
        msg.type = int(obs_info_one[9])

    pub.publish(msg_obstacles)    
    rate.sleep()