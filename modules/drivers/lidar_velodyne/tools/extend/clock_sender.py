 #!/usr/bin/env python  

import rospy
import rosgraph_msgs.msg
import time

if __name__ == '__main__':
    rospy.init_node('clock_pub')

    c = rosgraph_msgs.msg.Clock()
    c.clock.secs = 1521101066
    c.clock.nsecs = 0
    pub = rospy.Publisher('/clock', rosgraph_msgs.msg.Clock, queue_size=10)

    while True:
       pub.publish(c)
       time.sleep(0.1)
