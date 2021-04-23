#!/usr/bin/env python
import numpy as np
import rospy
from pb_msgs.msg import MonitorMessage
import sys, os, time
# from modules.common.monitor_log.proto.monitor_log_pb2 import MonitorMessage

def monitorCallback(monitorMessage):
    # get the monitor message item
    monitorMessageItem = monitorMessage.item[0]
    # extract message from the item
    msg = monitorMessageItem.msg
    # expected message when collision is detected
    collisionMessage = 'Found collision with obstacle: '

    # check if the message contains collision information
    if (collisionMessage in msg):
        # extract the obstacle id from the message
        # need to consider when id has '_0' suffixes which cannot be convert to int directly
        if ('_' in msg):
            # find the index of of '_' in the msg
            position = msg.find('_')
            # drop the chars include and after '_' to get a proper int
            msg = msg[:position]
        obstacle_id = int(msg.replace(collisionMessage, ''))
        obstacle = [obstacle_id]

        #print(obstacle_id)

        # add new obstacle data into the csv file, path is determined at the start of main
        with open(file_dest, 'a') as csv:
            np.savetxt(csv, obstacle, fmt='%d', delimiter=',')
       
        # print("Collision detect.")

        # terminate fuzzer or metamorphic script

        # exit once collision is detected
        # os._exit(1)

def listener():
    rospy.init_node('collision_detect', anonymous=True)
    rospy.Subscriber('/apollo/monitor', MonitorMessage, monitorCallback)
    rospy.spin()

if __name__ == '__main__':
    # check the argument vector to obtain the destination of the output file 
    if (sys.argv[1] == 'src'):
        file_dest = '/apollo/auto-test/data/collision.csv'
    elif (sys.argv[1] == 'follow'):
        file_dest = '/apollo/auto-test/data/collision_new.csv'
    else:
        print("Invalid arguments for collision_detect.py")
        sys.exit()

    time.sleep(3)
    listener()
