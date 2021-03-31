#!/usr/bin/env python
import numpy as np
import rospy
from pb_msgs.msg import MonitorMessage
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

        # add new obstacle data into the csv file
        with open('/apollo/auto-test/data/collision.csv', 'a') as csv:
            np.savetxt(csv, obstacle, fmt='%d', delimiter=',')
       
        # need to remove redundant ids 
        # non-trivial in this script, temporarily implement id removal
        # in metamorphic.py and can be optimised later 
    print('Process has not finish yet...')

def listener():
    rospy.init_node('collision_detect', anonymous=True)
    rospy.Subscriber('/apollo/monitor', MonitorMessage, monitorCallback)
    rospy.spin()

if __name__ == '__main__':
    listener()
