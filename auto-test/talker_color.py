#!/usr/bin/env python
#import roslib; roslib.load_manifest('beginner_tutorials')
import rospy
from std_msgs.msg import ColorRGBA
def talker():
    #pub = rospy.Publisher('chatter', String)
    pub = rospy.Publisher('chatter_color', ColorRGBA)
    rospy.init_node('talker_color')
    while not rospy.is_shutdown():
        pub.publish(a=1.0)
        rospy.sleep(1.0)
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
