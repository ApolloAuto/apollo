"""
this module creates a node and fake prediction data based
on json configurations
"""
import argparse
import math
import time

import numpy
import rospy
import simplejson
from std_msgs.msg import String

from modules.prediction.proto.prediction_obstacle_pb2 import PredictionObstacle
from modules.prediction.proto.prediction_obstacle_pb2 import PredictionObstacles


def prediction_publisher(prediction_topic, rate):
    """publisher"""
    pub = rospy.Publisher(prediction_topic, String, queue_size=1)
    rospy.init_node('prediction', anonymous=True)
    rate = rospy.Rate(rate)
    seq_num = 1
    while not rospy.is_shutdown():
        prediction = PredictionObstacles()
        prediction.header.sequence_num = seq_num
        prediction.header.timestamp_sec = rospy.Time.now().to_sec()
        prediction.header.module_name = "prediction"
        print str(prediction)
        s = String()
        s.data = prediction.SerializeToString()
        pub.publish(s)
        seq_num += 1
        rate.sleep()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="create empty prediction message",
            prog="replay_prediction.py")
    parser.add_argument("-t", "--topic", action="store", type=str, default="/pnc/prediction",
            help="set the prediction topic")
    parser.add_argument("-r", "--rate", action="store", type=int, default=10,
            help="set the prediction topic publish time duration")
    args = parser.parse_args()
    try:
        prediction_publisher(args.topic, args.rate)
    except rospy.ROSInterruptException:
        pass
