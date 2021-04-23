import rospy
import numpy as np

from pb_msgs.msg import RoutingResponse, RoutingRequest

def routingRequestCallback(RoutingRequest):
    print(RoutingRequest)
    print("==========================================================")

def listener_request():
    rospy.init_node('routing_request', anonymous=True)
    rospy.Subscriber('/apollo/routing_request', RoutingRequest, routingRequestCallback)
    rospy.spin()

if __name__ == '__main__':
    listener_request()
    # listener_respond()
