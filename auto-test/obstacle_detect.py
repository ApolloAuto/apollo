#!/usr/bin/env python

import rospy
from modules.perception.proto import perception_obstacle_pb2

def callback(data):
    print (data)
    print '-----------------'

def listener():
    rospy.init_node('obstacle_detect', anonymous=True)
    rospy.Subscriber('/apollo/perception/obstacles',perception_obstacle_pb2.PerceptionObstacles, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
