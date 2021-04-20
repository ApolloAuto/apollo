import rospy
from pb_msgs.msg import RoutingResponse, RoutingRequest

def routingRequestCallback(RoutingRequest):
    print(RoutingRequest)
    print("==========================================================")

def listener_request():
    rospy.init_node('routing_request', anonymous=True)
    rospy.Subscriber('/apollo/routing_request', RoutingRequest, routingRequestCallback)
    rospy.spin()

def routingRespondCallback(RoutingRespond):
    print(RoutingRespond)
    print("==========================================================")

def listener_respond():
    rospy.init_node('routing_respond', anonymous=True)
    rospy.Subscriber('/apollo/routing_respond', RoutingRespond, routingRespondCallback)
    rospy.spin()


if __name__ == '__main__':
    # listener_request()
    listener_respond()
