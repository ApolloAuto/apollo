"""
print received perception message
"""
import argparse
import math
import time

import numpy
from cyber_py import cyber
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
    cyber.init()
    node = cyber.Node("perception")
    node.create_reader(perception_topic, String, receiver)
    node.spin()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="create fake perception obstacles",
            prog="replay_perception.py")
    parser.add_argument("-t", "--topic", action="store", type=str, default="/perception/obstacles",
            help="set the perception topic")
            
    args = parser.parse_args()
    perception_receiver(args.topic)
