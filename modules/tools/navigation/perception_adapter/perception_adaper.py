#!/usr/bin/env python3

###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

import rospy
from modules.drivers.proto import mobileye_pb2
from modules.perception.proto import perception_obstacle_pb2

perception_pub = None


def mobileye_callback(mobileye_pb):
    perception_pb = perception_obstacle_pb2.PerceptionObstacles()

    lane_type_mapper = {}
    lane_type_mapper[0] = perception_obstacle_pb2.LaneMarker.LANE_TYPE_DASHED
    lane_type_mapper[1] = perception_obstacle_pb2.LaneMarker.LANE_TYPE_SOLID
    lane_type_mapper[2] = perception_obstacle_pb2.LaneMarker.LANE_TYPE_UNKNOWN
    lane_type_mapper[3] = perception_obstacle_pb2.LaneMarker.LANE_TYPE_ROAD_EDGE
    lane_type_mapper[4] = perception_obstacle_pb2.LaneMarker.LANE_TYPE_SOLID
    lane_type_mapper[5] = perception_obstacle_pb2.LaneMarker.LANE_TYPE_DASHED
    lane_type_mapper[6] = perception_obstacle_pb2.LaneMarker.LANE_TYPE_UNKNOWN

    perception_pb.lane_marker.left_lane_marker.lane_type = \
        lane_type_mapper.get(mobileye_pb.lka_766.lane_type,
                             perception_obstacle_pb2.LaneMarker.LANE_TYPE_UNKNOWN)
    perception_pb.lane_marker.left_lane_marker.quality = \
        mobileye_pb.lka_766.quality / 4.0
    perception_pb.lane_marker.left_lane_marker.model_degree = \
        mobileye_pb.lka_766.model_degree
    perception_pb.lane_marker.left_lane_marker.c0_position = \
        mobileye_pb.lka_766.position
    perception_pb.lane_marker.left_lane_marker.c1_heading_angle = \
        mobileye_pb.lka_767.heading_angle
    perception_pb.lane_marker.left_lane_marker.c2_curvature = \
        mobileye_pb.lka_766.curvature
    perception_pb.lane_marker.left_lane_marker.c3_curvature_derivative = \
        mobileye_pb.lka_766.curvature_derivative
    perception_pb.lane_marker.left_lane_marker.view_range = \
        mobileye_pb.lka_767.view_range

    perception_pb.lane_marker.right_lane_marker.lane_type = \
        lane_type_mapper.get(mobileye_pb.lka_768.lane_type,
                             perception_obstacle_pb2.LaneMarker.LANE_TYPE_UNKNOWN)
    perception_pb.lane_marker.right_lane_marker.quality = \
        mobileye_pb.lka_768.quality / 4.0
    perception_pb.lane_marker.right_lane_marker.model_degree = \
        mobileye_pb.lka_768.model_degree
    perception_pb.lane_marker.right_lane_marker.c0_position = \
        mobileye_pb.lka_768.position
    perception_pb.lane_marker.right_lane_marker.c1_heading_angle = \
        mobileye_pb.lka_769.heading_angle
    perception_pb.lane_marker.right_lane_marker.c2_curvature = \
        mobileye_pb.lka_768.curvature
    perception_pb.lane_marker.right_lane_marker.c3_curvature_derivative = \
        mobileye_pb.lka_768.curvature_derivative
    perception_pb.lane_marker.right_lane_marker.view_range = \
        mobileye_pb.lka_769.view_range

    perception_pub.publish(perception_pb)


def init():
    global perception_pub
    rospy.init_node("perception_adapter", anonymous=True)
    rospy.Subscriber('/apollo/sensor/mobileye',
                     mobileye_pb2.Mobileye,
                     mobileye_callback)
    perception_pub = rospy.Publisher("/apollo/perception/obstacles",
                                     perception_obstacle_pb2.PerceptionObstacles,
                                     queue_size=1)


if __name__ == '__main__':
    try:
        init()
        rospy.spin()
    finally:
        pass
