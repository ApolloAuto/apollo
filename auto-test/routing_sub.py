import rospy
import numpy as np

from pb_msgs.msg import RoutingResponse, RoutingRequest

def routingRequestCallback(RoutingRequest):
    # waypoints = RoutingRequest.waypoint
    # len_points = len(waypoints)

    # for point in range(len_points):
    #     curr_point = waypoints[point]
    #     print(curr_point.id)
    #     print(type(curr_point.id))
    #     routing_data = np.array([[curr_point.id, curr_point.s, curr_point.pose.x, curr_point.pose.y]])
    #     with open('/apollo/auto-test/data/waypoints.csv', 'a') as csv:
    #         np.savetxt(csv, routing_data, fmt = ['%s','%f','%f','%f'], delimiter=',',  encoding='utf8')
    print(RoutingRequest)
    print("==========================================================")

def listener_request():
    rospy.init_node('routing_request', anonymous=True)
    rospy.Subscriber('/apollo/routing_request', RoutingRequest, routingRequestCallback)
    rospy.spin()

if __name__ == '__main__':
    listener_request()
    # listener_respond()
