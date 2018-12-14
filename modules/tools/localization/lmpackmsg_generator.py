#!/usr/bin/python

from math import cos
from math import sin
import rosbag
import sys
from modules.localization.proto import odometry_lane_marker_pb2


def read_bag(filename):
    """Extraxt obstacle messages and odometry messages from bag file
    Args:
            filename: ROS Bag file name
    Returns:
            tuple: (obstaclemsg_list, odometrymsg_list)
    """
    obs_list = []      # obstacle message list
    odo_list = []      # odometry message list
    bag = rosbag.Bag(filename)
    for topic, msg, t in bag.read_messages(topics=['/apollo/perception/obstacles']):
        obs_list.append(msg)
    for topic, msg, t in bag.read_messages(topics=['/apollo/localization/pose']):
        odo_list.append(msg)
    bag.close()
    return (obs_list, odo_list)


def calculate_offset(theta, relative_x, relative_y):
    """Calculate flu to enu transform offset according to heading
    Args:
            theta: heading value
            relative_x: x value in relative
            relative_y: y value in relative
    Returns:
            tuple: (x_offset_value, y_offset_value)
    """
    cos_theta = cos(theta)
    sin_theta = sin(theta)
    return (relative_x * cos_theta + relative_y * sin_theta,
            - relative_x * sin_theta + relative_y * cos_theta)


def get_curve_value(x_value, c0, c1, c2, c3):
    """Get the curve y_value according to x_value and curve analysis formula
        y = c3 * x**3 + c2 * x**2 + c1 * x + c0
    Args:
            x_value: value of x
            c3: curvature_derivative
            c2: curvature
            c1: heading_angle
            c0: position
    Returns:
            y_value according to the analysis formula with x = x_value
    """
    return c3 * (x_value ** 3) + c2 * (x_value ** 2) + c1 * x_value + c0


def calculate_derivative(x_value, c0, c1, c2, c3):
    """Get the first derivative value according to x_value and curve analysis formula
        y = c3 * x**3 + c2 * x**2 + c1 * x + c0
    Args:
            x_value: value of x
            c3: curvature_derivative
            c2: curvature
            c1: heading_angle
            c0: position
    Returns:
            the first derivative value when x equal to x_value
    """
    return 3 * c3 * x_value ** 2 + 2 * c2 + c1


def calculate_curvity(x_value, c0, c1, c2, c3):
    """Get the curvity value according to x_value and curve analysis formula
        y = c3 * x**3 + c2 * x**2 + c1 * x + c0
    Args:
            x_value: value of x
            c3: curvature_derivative
            c2: curvature
            c1: heading_angle
            c0: position
    Returns:
            K = |y''| / (1 + y'**2)**(3.0/2)
            curvity_value K according to the analysis formula with x = x_value
    """
    return abs(6 * c3 * x_value + 2 * c2) / (
        (1 + calculate_derivative(x_value, c0, c1, c2, c3) ** 2) ** (3.0/2))


def cal_proportion(source_tuple, obs_index):
    """calculate the proportion of two gps localization proportion
     according to their timestamp and obstacle timestamp and 
     get the suitable odo_index whose timestamp is nearest to 
     the desired obstacle's timestamp
    Args:
        source_tuple:(obstaclemsg_list, odometrymsg_list)
    Returns:
        tuple: (proportion, odo_index)    
    """
    c = source_tuple[0][obs_index].header.timestamp_sec
    for odo_index in range(len(source_tuple[1]) - 1):
        a = source_tuple[1][odo_index].header.timestamp_sec
        b = source_tuple[1][odo_index + 1].header.timestamp_sec
        if(a < c and c < b):
            return ((b - c)/(b - a), odo_index)
        elif(c == a):
            return (1, odo_index)
        elif (c == b):
            return (0, odo_index)
        else:
            continue
    return(1, odo_index)


def write_info(source_tuple, filename):
    """1. acquire the correct gps msg from odometrymsg according to the
              timestamp from obstaclemsg
       2. set the value of properties of OdometryLaneMarker message
       3. group the OdometryLaneMarker message to ContourOdometryLaneMarkers
       4. set the value of properties of OdometryLaneMarkersPack message
       5. write the serialized string of OdometryLaneMarkersPack message to file
    Args:
            source_tuple:(obstaclemsg_list, odometrymsg_list)
            filename: file to store protobuf msg
    """
    f = open(filename, "wb")
    odometry_lane_markers_pack = odometry_lane_marker_pb2.OdometryLaneMarkersPack()
    left_lane_marker_group = odometry_lane_markers_pack.lane_markers.add()
    right_lane_marker_group = odometry_lane_markers_pack.lane_markers.add()
    # each message in obstacle message list
    for obs_index in range(len(source_tuple[0])):
        proportion, odo_index = cal_proportion(source_tuple, obs_index)
        localization_former = source_tuple[1][odo_index].pose.position
        localization_next = source_tuple[1][odo_index + 1].pose.position
        vehicle_heading_former = source_tuple[1][odo_index].pose.heading
        vehicle_heading_next = source_tuple[1][odo_index + 1].pose.heading
        heading_value = (vehicle_heading_former * proportion +
                         vehicle_heading_next * (1 - proportion))
        initial_x = (localization_former.x * proportion +
                     localization_next.x * (1 - proportion))
        initial_y = (localization_former.y * proportion +
                     localization_next.y * (1 - proportion))
        initial_z = (localization_former.z * proportion +
                     localization_next.z * (1 - proportion))

        lmd_left_lane_marker = left_lane_marker_group.lane_marker.add()
        left_c0_position = (
            source_tuple[0][obs_index].lane_marker.left_lane_marker.c0_position)
        left_c1_heading_angle = (
            source_tuple[0][obs_index].lane_marker.left_lane_marker.c1_heading_angle)
        left_c2_curvature = (
            source_tuple[0][obs_index].lane_marker.left_lane_marker.c2_curvature)
        left_c3_curvature_derivative = (
            source_tuple[0][obs_index].lane_marker.left_lane_marker.c3_curvature_derivative)
        left_insert_point = True
        for i in range(10):
            location_offset = calculate_offset(- heading_value, i * 0.15, get_curve_value(
                i * 0.15, left_c0_position, left_c1_heading_angle, left_c2_curvature, left_c3_curvature_derivative))
            px = initial_x + location_offset[0]
            py = initial_y + location_offset[1]
            pz = initial_z
            for lane in left_lane_marker_group.lane_marker:
                for point in lane.points:
                    if(abs(point.position.x - px) < 0.05 and abs(point.position.y - py) < 0.05):
                        left_insert_point = False
                        point.position.x = (point.position.x + px) / 2.0
                        point.position.y = (point.position.y + py) / 2.0
                        point.position.z = (point.position.z + pz) / 2.0
            if(left_insert_point):
                point = lmd_left_lane_marker.points.add()
                point.position.x = px
                point.position.y = py
                point.position.z = pz
                direct_offset = calculate_offset(- heading_value, 1.0, 0.0)
                point.direct.x = direct_offset[0]
                point.direct.y = direct_offset[1]
                point.direct.z = 0.0
                point.curvature = calculate_curvity(
                    i * 0.15, left_c0_position, left_c1_heading_angle, left_c2_curvature, left_c3_curvature_derivative)

        lmd_right_lane_marker = right_lane_marker_group.lane_marker.add()
        right_c0_position = (
            source_tuple[0][obs_index].lane_marker.right_lane_marker.c0_position)
        right_c1_heading_angle = (
            source_tuple[0][obs_index].lane_marker.right_lane_marker.c1_heading_angle)
        right_c2_curvature = (
            source_tuple[0][obs_index].lane_marker.right_lane_marker.c2_curvature)
        right_c3_curvature_derivative = (
            source_tuple[0][obs_index].lane_marker.right_lane_marker.c3_curvature_derivative)
        right_insert_point = True
        for i in range(10):
            location_offset = calculate_offset(- heading_value, i * 0.15, get_curve_value(
                i * 0.15, right_c0_position, right_c1_heading_angle, right_c2_curvature, right_c3_curvature_derivative))
            px = initial_x + location_offset[0]
            py = initial_y + location_offset[1]
            pz = initial_z
            for lane in right_lane_marker_group.lane_marker:
                for point in lane.points:
                    if(abs(point.position.x - px) < 0.05 and abs(point.position.y - py) < 0.05):
                        right_insert_point = False
                        point.position.x = (point.position.x + px) / 2.0
                        point.position.y = (point.position.y + py) / 2.0
                        point.position.z = (point.position.z + pz) / 2.0
            if(right_insert_point):
                point = lmd_right_lane_marker.points.add()
                point.position.x = px
                point.position.y = py
                point.position.z = pz
                direct_offset = calculate_offset(- heading_value, 1.0, 0.0)
                point.direct.x = direct_offset[0]
                point.direct.y = direct_offset[1]
                point.direct.z = 0.0
                point.curvature = calculate_curvity(
                    i * 0.15, right_c0_position, right_c1_heading_angle, right_c2_curvature, right_c3_curvature_derivative)
    f.write(odometry_lane_markers_pack.SerializeToString())
    f.close()


def main():
    """main entry of lmpackmsg_generator
    Args:
        argv[1]:raw rosbag input to generate lanemarker msgs
        argv[2]:filename of generated lane marker msgs
    """
    if(len(sys.argv) != 3):
        print(
            "Please provide --argv[1]:raw rosbag input to generate lanemarker msgs --argv[2]:filename of generated lane marker msgs")
        sys.exit()
    resource = read_bag(sys.argv[1])
    write_info(resource, sys.argv[2])


if __name__ == '__main__':
    main()
