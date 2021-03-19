#!/usr/bin/env python
#import roslib; roslib.load_manifest('beginner_tutorials')
import rospy
from std_msgs.msg import ColorRGBA
def callback(data):
    rospy.loginfo(rospy.get_name()+ "I heard r=%s g=%s b=%s a=%s", data.r, data.g, data.b, data.a)

def listener():
    rospy.init_node('listener_color', anonymous=True)
    rospy.Subscriber("chatter_color", ColorRGBA, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
