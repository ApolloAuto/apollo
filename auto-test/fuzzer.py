import random, time
import numpy as np
from numpy import arctan, cos, sin, genfromtxt
import rospy
from routing_pub import talker as rtalker
from pb_msgs.msg import PerceptionObstacles
from modules.routing.proto import routing_pb2


# calculate the tansformed coordinates of x and y
# according to the angle and distance between center to origin
def transform(x, y, angle, x_center, y_center, compress_x_y):
    # first translate to the origin
    x = x - x_center
    y = y - y_center
	
    # compress the width of the region
    if (compress_x_y == 0):
        x = 0.2 * x
    if (compress_x_y == 1):
        y = 0.2 * y

    # apply rotation
    x = x * cos(angle) - y * sin(angle)
    y = x * sin(angle) + y * cos(angle)

    # translate back to the previous position
    x_transform = x + x_center
    y_transform = y + y_center

    return x_transform, y_transform
	

def talker():
    pub_obstacle = rospy.Publisher('/apollo/perception/obstacles', PerceptionObstacles, queue_size=10)
    pub_routing = rospy.Publisher('/apollo/routing_request', routing_pb2.RoutingRequest, queue_size=10)

    rospy.init_node('talker', anonymous=True)

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

    # define the frequency of refresh (Hz)
    rate = rospy.Rate(0.2)

    # read data from routings.csv
    routings = genfromtxt('/apollo/auto-test/data/routings.csv', dtype=int, delimiter=',')
    # select the routing by id
    routing_id = 5 # 'Sv sunrise loop'
    # routing = routings[routings[:,0] == routing_id]
    routing = routings[routings[:,0] == routing_id][0]
    # extract the start and end coordinates from the list
    start_x = routing[1.1]
    start_y = routing[2]
    end_x = routing[3]
    end_y = routing[4]


    # define the boundary of region where obstacles are generated
    # using the start and end points in this case
    bound_left = min(end_x, start_x)
    bound_right = max(end_x, start_x)
    bound_up = max(end_y, start_y)
    bound_down = min(end_y, start_y)

    center_x = 0.5 * (start_x + end_x)
    center_y = 0.5 * (start_y + end_y)

    # get the tan value of angle of the road according to the horizontal line
    tan_value = abs(float(end_y-start_y) / float(end_x-start_x))
    # get the angle 
    # road_angle = arctan(tan_value)
    # print(road_angle)

    # determine whether the region is general horizontal or vertical
    if abs(start_x-end_x) > abs(start_y-end_y):
    	compress = 1
    	angle = arctan(tan_value)
        # negate the angle if counter-clockwise
        if (start_x-end_x)*(start_y-end_y) < 0:
            angle = -angle
    else:
    	compress = 0
        angle = arctan(1/tan_value)
        # negate the angle if counter-clockwise
        if (start_x-end_x)*(start_y-end_y) > 0:
            angle = -angle

    # calculate the area of the region
    area_region = (bound_right - bound_left) * (bound_up - bound_down)
    # define the obstacle density (0 - 1)
    obstacle_density = 0.003

    # define number of obstacles
    n_obstacles = int(obstacle_density * area_region)

    # set scenario id to 1
    # scenario_id = 1
    # add a large number to obstacle id to make them unique among all simulation scenarios
    id_prefix = 10000
    # denfine number of scenarios
    scenario_num = 6

    for scenario_id in range(1, scenario_num+1):
        # create PerceptionObstacles object
        msg_obstacles = PerceptionObstacles()
        # print('Scenario/ ID: %d' % scenario_id)
        for obs in range(n_obstacles):
            msg = msg_obstacles.perception_obstacle.add()
            msg.id = obs + scenario_id * id_prefix
            # randomly assign x and y coordinates to the obstacle
            x = random.uniform(bound_left, bound_right)
            y = random.uniform(bound_down, bound_up)
			
            # apply transformation to fit the region to the routing
            x_, y_ = transform(x, y, angle, center_x, center_y, compress)
	   
            msg.position.x = x_
            msg.position.y = y_
            msg.position.z = 0

            # assign random theta to the obstacle
            # theta represents the heading direction of the obstacle
            msg.theta = 1.0

            # custom obstacle dimentions
            msg.length = random.uniform(1, 4)
            msg.width = random.uniform(1, 4)
            msg.height = random.uniform(1, 4)

            # define the type of the obstacle (default 10: general obstacle)
            msg.type = random.randrange(0, 5)

            # gather the obstacle information in array
            # id, scenario_id, position_x, position_y, position_z, direction, length, width, height, type
            obstacle_data = np.array([[msg.id, scenario_id, msg.position.x, msg.position.y,
                             msg.position.z, msg.theta, msg.length, msg.width, msg.height, msg.type]])
	    
            # the reason why obstacle_data is a 2D array is that using a 1D array will insert a column of data 
            # to the file rather than a row. Thus, this is a hack to fix the bug.
				
            # add new obstacle data into the csv file
            with open('/apollo/auto-test/data/obstacles.csv', 'a') as csv:
                np.savetxt(csv, obstacle_data, fmt = ['%d','%d','%.4f','%.4f','%.4f','%.4f','%.4f','%.4f','%.4f','%d'], delimiter=',')

        # increment the scenario counter
        scenario_id = scenario_id + 1
        
        # publish the obstacles to ROS topic '/apollo/perception/obstacles'
        pub_obstacle.publish(msg_obstacles)
        rate.sleep()

    # # publish an empty set of obstacles to reset the scenario for the next test case
    # msg_obstacles = PerceptionObstacles()
    # pub_obstacle.publish(msg_obstacles)



if __name__ == '__main__':
    try:
        talker()

    except rospy.ROSInterruptException:
        pass
