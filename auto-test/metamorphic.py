import numpy as np
from numpy import genfromtxt, arctan, cos, sin
import os, sys, time
from routing_pub import talker as rtalker
import rospy
from pb_msgs.msg import PerceptionObstacles
from modules.routing.proto import routing_pb2


# load data from data/obstacles.csv and data/collision.csv
obstacles_data = genfromtxt('/apollo/auto-test/data/obstacles.csv', delimiter=',')

# check whether collision.csv exists, it should not exist if no collision is detected
if (not os.path.exists('/apollo/auto-test/data/collision.csv')):
    print('No collision detected in the simulation!')
    sys.exit()

collision_data = genfromtxt('/apollo/auto-test/data/collision.csv', delimiter=',')

# remove duplicated data in collision_data
collision_data = np.unique(collision_data)
collision_id = collision_data[0]
print(collision_id)

# # find the information of all the collision obstacles in obstacle_data
# collision_obstacle_data = np.zeros([collision_size, 10])
# for c in range(collision_size):
#     obstacle_info = obstacles_data[obstacles_data[:,0] == collision_data[c]]
#     collision_obstacle_data[c] = obstacle_info[0]



# print collision rate
# get the number of scenarios generated
scenario_number = int(obstacles_data[-1][1])
obstacle_number = int(obstacles_data.shape[0])
# check whether Obstacle number is divisable by scenario number
if (obstacle_number % scenario_number != 0):
    print("Error: Obstacle number is not divisable by scenario number! Something goes wrong!")
    sys.exit()


obstacle_per_scenario = obstacle_number / scenario_number

# collision_analysis = np.array([[obstacles_data.shape[0], scenario_number, collision_size, general_rate]])
# with open('/apollo/auto-test/data/collision_analysis.csv', 'a') as csv:
#     np.savetxt(csv, collision_analysis, fmt=['%d','%d','%d','%.6f'], delimiter=',')

# regenerate the follow-up scenarios based on the obstacle data and MR
# MR: Suppose that in a driving scenario,S, a car collided with a static obstacle O at location L, 
#     Construct a follow-up driving scenario,S', that is identical to S except that O is repositioned 
#     slightly further away from the previous position. Run the follow-up driving scenario. The car 
#     should not stop before L.

# generate the obstacles based on obstacle.csv
pub_obstacle = rospy.Publisher('/apollo/perception/obstacles', PerceptionObstacles, queue_size=10)
pub_routing = rospy.Publisher('/apollo/routing_request', routing_pb2.RoutingRequest, queue_size=10)
rospy.init_node('talker', anonymous=True)
# define the frequency of refresh (Hz)
rate = rospy.Rate(0.2)

# send routing request
sequence_num = 0

msg_routing_request = routing_pb2.RoutingRequest()
msg_routing_request.header.timestamp_sec = rospy.get_time()
msg_routing_request.header.module_name = 'routing_request'
msg_routing_request.header.sequence_num = sequence_num
sequence_num = sequence_num + 1

# load data from data/waypoints.csv
routing_data = genfromtxt('/apollo/auto-test/data/waypoints.csv', delimiter=',',dtype=None)
len_waypoints = len(routing_data)

for point in range(len_waypoints):
    curr_point = routing_data[point]
    msg = msg_routing_request.waypoint.add()
    msg.id = curr_point[0]
    msg.s = curr_point[1]
    msg.pose.x = curr_point[2]
    msg.pose.y = curr_point[3]

# print(msg_routing_request)
# wait for 2 seconds to let the message published successfully
# if time is too short, the message may be ommited by the system
time.sleep(2.0)
pub_routing.publish(msg_routing_request)

road_angle = 1.33015289662

# generate the same number of scenarios 
for scenario in range(1, scenario_number+1):
    # print('Scenario ID: %d' % scenario)
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
        # todo move obstacle further away
        # if (int(obs_info_one[0]) == collision_id):
        #     msg.position.x = msg.position.x - 4.0 * cos(road_angle)
        #     msg.position.y = msg.position.y - 4.0 * sin(road_angle)

        msg.theta = obs_info_one[5]
        msg.length = obs_info_one[6]
        msg.width = obs_info_one[7]
        msg.height = obs_info_one[8]
        msg.type = int(obs_info_one[9])

    pub_obstacle.publish(msg_obstacles)    
    rate.sleep()

# # publish an empty set of obstacles to reset the scenario for the next test case
# msg_obstacles = PerceptionObstacles()
# pub_obstacle.publish(msg_obstacles)

if (not os.path.exists('/apollo/auto-test/data/collision_new.csv')):
    print('No collision detected in the follow-up simulation!')
    sys.exit()

collision_data_new = genfromtxt('/apollo/auto-test/data/collision_new.csv', delimiter=',')

# remove duplicated data in collision_data
collision_data_new = np.unique(collision_data_new)

print('collosions 1: ', collision_data)
print('collosions 2: ', collision_data_new)

collision_cmp = np.array([collision_data, collision_data_new])
# add new obstacle data into the csv file, path is determined at the start of main
with open('/apollo/auto-test/data/collision_compare.csv', 'a') as csv:
    np.savetxt(csv, collision_cmp, fmt='%d', delimiter=',')

if (np.array_equal(collision_data, collision_data_new)):
    print('Identical repruduction!')
    
print('================================================================')