#!/usr/bin/env python

import rospy
from modules.planning.proto import planning_pb2


def callback(data):
    print data
    print '-----------------'

def listener():
    rospy.init_node('collision_detect', anonymous=True)
    rospy.Subscriber('/apollo/planning',planning_pb2.ADCTrajectoryPoint, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
