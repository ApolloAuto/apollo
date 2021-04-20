import rospy
import time
from pb_msgs.msg import RoutingResponse, RoutingRequest
from modules.routing.proto import routing_pb2

def talker():
    pub = rospy.Publisher('/apollo/routing_request', routing_pb2.RoutingRequest, queue_size=1)
    rospy.init_node('routing_request', anonymous=True)

    sequence_num = 0

    msg_routing_request = routing_pb2.RoutingRequest()
    # msg_routing_request.broadcast = True
    msg_routing_request.header.timestamp_sec = rospy.get_time()
    msg_routing_request.header.module_name = 'routing_request'
    msg_routing_request.header.sequence_num = sequence_num
    sequence_num = sequence_num + 1

    msg = msg_routing_request.waypoint.add()
    msg.id = '1-1'
    msg.s = 1
    msg.pose.x = 587714
    msg.pose.y = 4141426

    msg = msg_routing_request.waypoint.add()
    msg.id = '1-2'
    msg.s = 80
    msg.pose.x = 587624
    msg.pose.y = 4141060

    msg = msg_routing_request.waypoint.add()
    msg.id = '1-3'
    msg.s = 100
    msg.pose.x = 587534
    msg.pose.y = 4140694

    print(msg_routing_request)
    time.sleep(2.0)

    pub.publish(msg_routing_request)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

# import argparse
# import atexit
# import os
# import sys
# import time

# import rospy
# import scipy.signal as signal
# from numpy import genfromtxt

# from pb_msgs.msg import RoutingRequest
# from modules.routing.proto import routing_pb2

# def main():
#     """
#     Main rosnode
#     """
#     rospy.init_node('mock_routing_requester', anonymous=True)
#     sequence_num = 0

#     routing_request = routing_pb2.RoutingRequest()
#     routing_request.header.timestamp_sec = rospy.get_time()
#     routing_request.header.module_name = 'routing_request'
#     routing_request.header.sequence_num = sequence_num
#     sequence_num = sequence_num + 1

#     waypoint = routing_request.waypoint.add()
#     waypoint.pose.x = 587696.82286
#     waypoint.pose.y = 4141446.66696
#     waypoint.id = '1-1'
#     waypoint.s = 1

#     waypoint = routing_request.waypoint.add()
#     waypoint.pose.x = 586948.740120
#     waypoint.pose.y = 4141171.118641
#     waypoint.id = '1-2'
#     waypoint.s = 80

#     print(routing_request)
#     request_publisher = rospy.Publisher(
#             '/apollo/routing_request', routing_pb2.RoutingRequest, queue_size=10)
#     time.sleep(2.0)
#     request_publisher.publish(routing_request)


# if __name__ == '__main__':
#     main()

