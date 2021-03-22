#!/usr/bin/env python

import rospy
from pb_msgs.msg import MonitorMessage
# from modules.common.monitor_log.proto.monitor_log_pb2 import MonitorMessage

def monitorCallback(data):
    print(type(data))
    print(type(data.item))
    print(data.item.msg)
    print '==========================='
def listener():
    rospy.init_node('collision_detect', anonymous=True)
    rospy.Subscriber('/apollo/monitor', MonitorMessage, monitorCallback)
    rospy.spin()

if __name__ == '__main__':
    listener()
