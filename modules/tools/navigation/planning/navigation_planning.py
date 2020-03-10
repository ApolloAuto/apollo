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
import datetime
import os
import time
import sys
import gflags
from gflags import FLAGS
from std_msgs.msg import String

from modules.drivers.proto import mobileye_pb2
from modules.planning.proto import planning_pb2
from modules.canbus.proto import chassis_pb2
from modules.localization.proto import localization_pb2
from modules.map.relative_map.proto import navigation_pb2
from path_decider import PathDecider
from speed_decider import SpeedDecider
from trajectory_generator import TrajectoryGenerator
from provider_mobileye import MobileyeProvider
from provider_chassis import ChassisProvider
from provider_localization import LocalizationProvider
from provider_routing import RoutingProvider
from obstacle_decider import ObstacleDecider
from ad_vehicle import ADVehicle

gflags.DEFINE_integer('max_cruise_speed', 20,
                      'max speed for cruising in meter per second')
gflags.DEFINE_boolean('enable_follow', False,
                      'enable follow function.')
gflags.DEFINE_boolean('enable_nudge', True,
                      'enable nudge function.')
gflags.DEFINE_boolean('enable_change_lane', False,
                      'enable change lane function.')
gflags.DEFINE_boolean('enable_routing_aid', True,
                      'enable planning leveraging routing information.')
gflags.DEFINE_string('navigation_planning_node_name', 'navigation_planning',
                     'node name for navigation planning.')
gflags.DEFINE_string('navigation_planning_topic', '/apollo/planning',
                     'navigation planning publish topic.')

planning_pub = None
log_file = None
path_decider = None
speed_decider = None
traj_generator = None
mobileye_provider = None
routing_provider = None
obs_decider = None
adv = None


def routing_callback(routing_str):
    routing_provider.update_navigation(routing_str)


def localization_callback(localization_pb):
    adv.update_localization(localization_pb)


def chassis_callback(chassis_pb):
    adv.update_chassis(chassis_pb)


def mobileye_callback2(mobileye_pb):
    if not adv.is_ready():
        return

    start_timestamp = time.time()
    mobileye_provider.update(mobileye_pb)
    mobileye_provider.process_obstacles()

    path_x, path_y, path_length = path_decider.get(mobileye_provider,
                                                   routing_provider,
                                                   adv)

    speed, final_path_length = speed_decider.get(mobileye_provider,
                                                 adv,
                                                 path_length)

    adc_trajectory = traj_generator.generate(path_x, path_y, final_path_length,
                                             speed,
                                             start_timestamp=start_timestamp)
    planning_pub.publish(adc_trajectory)
    log_file.write("duration: " + str(time.time() - start_timestamp) + "\n")


def mobileye_callback(mobileye_pb):
    if not adv.is_ready():
        return

    start_timestamp = time.time()
    mobileye_provider.update(mobileye_pb)
    mobileye_provider.process_obstacles()
    obs_decider.update(mobileye_provider)

    path = path_decider.get_path(mobileye_provider, routing_provider, adv,
                                 obs_decider)

    speed, final_path_length = speed_decider.get(mobileye_provider,
                                                 adv,
                                                 path.range())

    adc_trajectory = traj_generator.generate(path, final_path_length,
                                             speed,
                                             start_timestamp=start_timestamp)
    planning_pub.publish(adc_trajectory)
    log_file.write("duration: " + str(time.time() - start_timestamp) + "\n")


def init():
    global planning_pub, log_file
    global path_decider, speed_decider, traj_generator, obs_decider
    global mobileye_provider, chassis_provider
    global localization_provider, routing_provider
    global adv

    path_decider = PathDecider(FLAGS.enable_routing_aid,
                               FLAGS.enable_nudge,
                               FLAGS.enable_change_lane)
    speed_decider = SpeedDecider(FLAGS.max_cruise_speed,
                                 FLAGS.enable_follow)
    traj_generator = TrajectoryGenerator()
    mobileye_provider = MobileyeProvider()
    routing_provider = RoutingProvider()
    adv = ADVehicle()
    obs_decider = ObstacleDecider()

    pgm_path = os.path.dirname(os.path.realpath(__file__))
    log_path = pgm_path + "/logs/"
    if not os.path.exists(log_path):
        os.makedirs(log_path)
    now = datetime.datetime.now().strftime("%Y-%m-%d_%H.%M.%S")
    log_file = open(log_path + now + ".txt", "w")

    rospy.init_node(FLAGS.navigation_planning_node_name, anonymous=True)
    rospy.Subscriber('/apollo/sensor/mobileye',
                     mobileye_pb2.Mobileye,
                     mobileye_callback)
    rospy.Subscriber('/apollo/localization/pose',
                     localization_pb2.LocalizationEstimate,
                     localization_callback)
    rospy.Subscriber('/apollo/canbus/chassis',
                     chassis_pb2.Chassis,
                     chassis_callback)
    rospy.Subscriber('/apollo/navigation',
                     navigation_pb2.NavigationInfo, routing_callback)
    planning_pub = rospy.Publisher(FLAGS.navigation_planning_topic,
                                   planning_pb2.ADCTrajectory, queue_size=1)


if __name__ == '__main__':

    try:
        argv = FLAGS(sys.argv)  # parse flags
    except gflags.FlagsError as e:
        print('%s\\nUsage: %s ARGS\\n%s' % (e, sys.argv[0], FLAGS))
        sys.exit(1)
    try:
        init()
        rospy.spin()
    finally:
        log_file.close()
