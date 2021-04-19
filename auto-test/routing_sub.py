import rospy
from pb_msgs.msg import RoutingResponse, RoutingRequest

def routingCallback(RoutingRequest):
    print(RoutingRequest)
    print("==========================================================")

def listener():
    rospy.init_node('routings', anonymous=True)
    rospy.Subscriber('/apollo/routing_request', RoutingRequest, routingCallback)
    rospy.spin()

if __name__ == '__main__':
    listener()
