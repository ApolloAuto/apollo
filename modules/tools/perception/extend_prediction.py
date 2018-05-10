"""
print received prediction message
"""
import argparse
import math
import sys
import time

import numpy
import rospy
from std_msgs.msg import String

import common.proto_utils as proto_utils
from modules.prediction.proto.prediction_obstacle_pb2 import PredictionObstacle
from modules.prediction.proto.prediction_obstacle_pb2 import PredictionObstacles


def distance(p1, p2):
    """distance between two trajectory points"""
    return math.sqrt((p1.y - p2.y)**2 + (p1.x - p2.x)**2);


def get_trajectory_length(trajectory):
    """get_trajectory_length"""
    total = 0.0
    for i in range(1, len(trajectory.trajectory_point)):
        total += distance(trajectory.trajectory_point[i - 1],
                trajectory.trajectory_point[i])
    return total


def extend_prediction(prediction, min_length, min_time):
    """extend prediction"""
    obstacles = len(prediction.prediction_obstacle)
    for obstacle in prediction.prediction_obstacle:
        i = 0;
        for trajectory in obstacle.trajectory:
            points = trajectory.trajectory_point
            point_num = len(points)
            trajectory_length = get_trajectory_length(trajectory)
            print >> sys.stderr, "%s trajectory %s length %s" % (obstacle.perception_obstacle.id, i, trajectory_length)
            i += 1
            if trajectory_length < min_length:
                second_last = points[point_num - 2]
                last_point = points[point_num - 1]
                x_diff = last_point.x - second_last.x
                y_diff = last_point.y - second_last.y
                t_diff = last_point.t - second_last.t
                delta_diff = math.sqrt(x_diff ** 2 + y_diff ** 2)
                cur_len = trajectory_length
                while cur_len < min_length:
                    last_point.x += x_diff
                    last_point.y += y_diff
                    last_point.t += t_diff
                    p = points.add()
                    p.CopyFrom(last_point)
                    last_point = p
                    cur_len += delta_diff
    return prediction


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="extend prediction trajectory")
    parser.add_argument("prediction", action="store", type=str, help="set the prediction file")
    parser.add_argument("-p", "--period", action="store", type=float, default=10.0,
            help="set the prediction period")
    parser.add_argument("-d", "--distance", action="store", type=float, default=70.0,
            help="set the prediction distance")
    args = parser.parse_args()
    prediction_data = proto_utils.get_pb_from_file(args.prediction,
                                                   PredictionObstacles())
    extended_prediction = extend_prediction(prediction_data, args.distance, args.period)
    print extended_prediction
