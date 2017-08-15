"""
print received perception message
"""
import argparse
import math
import time

import numpy
import rospy
from std_msgs.msg import String

from modules.perception.proto.perception_obstacle_pb2 import PerceptionObstacle
from modules.perception.proto.perception_obstacle_pb2 import PerceptionObstacles


def receiver(data):
    """receiver"""
    perception = PerceptionObstacles()
    perception.ParseFromString(data.data)
    print str(perception)


def perception_receiver(perception_topic):
    """publisher"""
    rospy.init_node('perception', anonymous=True)
    pub = rospy.Subscriber(perception_topic, String, receiver)
    rospy.spin()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="create fake perception obstacles",
            prog="replay_perception.py")
    parser.add_argument("-t", "--topic", action="store", type=str, default="/perception/obstacles",
            help="set the perception topic")
    parser.add_argument("-p", "--period", action="store", type=float, default=0.1,
            help="set the perception topic publish time duration")
    args = parser.parse_args()
    perception_receiver(args.topic)
