#!/usr/bin/env python

import rospy
from pb_msgs.msg import MonitorMessage
# from modules.common.monitor_log.proto.monitor_log_pb2 import MonitorMessage

def monitorCallback(monitorMessage):
    # get the monitor message item
    monitorMessageItem = monitorMessage.item[0]
    # extract message from the item
    msg = monitorMessageItem.msg
    # check if the message contains collision information
    collisionMessage = 'Found collision with obstacle'
    if (collisionMessage in msg):
        print(msg)
    
def listener():
    rospy.init_node('collision_detect', anonymous=True)
    rospy.Subscriber('/apollo/monitor', MonitorMessage, monitorCallback)
    rospy.spin()

if __name__ == '__main__':
    listener()
