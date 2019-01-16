#!/usr/bin/python

from math import cos
from math import sin
from modules.localization.proto import odometry_lane_marker_pb2
from sys import argv
from sys import exit
import matplotlib.pyplot as plt
import numpy as np


def read_lane_markers_data(filename):
    left_lane_markers_points_group = []
    right_lane_markers_points_group = []
    input = open(filename, 'rb')
    data = input.read()
    target = odometry_lane_marker_pb2.OdometryLaneMarkersPack()
    target.ParseFromString(data)
    left_lane_marker_group = target.lane_markers[0]
    right_lane_marker_group = target.lane_markers[1]
    for lane_marker in left_lane_marker_group.lane_marker:
        for point in lane_marker.points:
            left_lane_markers_points_group.append(
                [point.position.x, point.position.y])

    for lane_marker in right_lane_marker_group.lane_marker:
        for point in lane_marker.points:
            right_lane_markers_points_group.append(
                [point.position.x, point.position.y])
    plot_lane(left_lane_markers_points_group, right_lane_markers_points_group)


def plot_lane(left_points, right_points):
    """plot the points list in matplotlib 
    Args:
        left_points: points list of left lane marker group
        right_points: points list of right lane marker group
    """
    x_left = np.array(left_points)[:, 0]
    y_left = np.array(left_points)[:, 1]
    plt.scatter(x_left, y_left)
    x_right = np.array(right_points)[:, 0]
    y_right = np.array(right_points)[:, 1]
    plt.scatter(x_right, y_right)
    plt.show()


def main():
    """main entry of lane_markers_displayer
    Args:
        argv[1]:raw OdometryLanemarker bin file input to display 
    """
    if(len(argv) != 2):
        print(
            "Please provide --argv[1]:raw OdometryLanemarker bin file to display")
        exit()
    read_lane_markers_data(argv[1])


if __name__ == '__main__':
    main()
