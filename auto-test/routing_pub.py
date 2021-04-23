import rospy
import time
from numpy import genfromtxt
from pb_msgs.msg import RoutingResponse, RoutingRequest
from modules.routing.proto import routing_pb2

def talker():
    pub = rospy.Publisher('/apollo/routing_request', routing_pb2.RoutingRequest, queue_size=1)
    # rospy.init_node('routing_request', anonymous=True)

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

    pub.publish(msg_routing_request)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
