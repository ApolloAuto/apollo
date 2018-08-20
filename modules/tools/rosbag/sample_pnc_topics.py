#!/usr/bin/env python

###############################################################################
# Copyright 2018 The Apollo Authors. All Rights Reserved.
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
"""
Sample PNC topics. For each /path/to/a.bag, will generate
/path/to/pnc_sample/a.bag.

Usage:
    ./sample_pnc_topics.py <bag_path>
        <bag_path>    Support * and ?.
Example:
    ./sample_pnc_topics.py '/mnt/nfs/public_test/2018-04-??/*/mkz8/*/*.bag'
"""

import glob
import os
import sys

import glog
import rosbag


class SamplePNC(object):
    """Sample bags to contain PNC related topics only."""
    TOPICS = [
        '/apollo/sensor/conti_radar',
        '/apollo/sensor/delphi_esr',
        '/apollo/sensor/gnss/best_pose',
        '/apollo/sensor/gnss/corrected_imu',
        '/apollo/sensor/gnss/gnss_status',
        '/apollo/sensor/gnss/imu',
        '/apollo/sensor/gnss/ins_stat',
        '/apollo/sensor/gnss/odometry',
        '/apollo/sensor/gnss/rtk_eph',
        '/apollo/sensor/gnss/rtk_obs',
        '/apollo/sensor/mobileye',
        '/apollo/canbus/chassis',
        '/apollo/canbus/chassis_detail',
        '/apollo/control',
        '/apollo/control/pad',
        '/apollo/navigation',
        '/apollo/perception/obstacles',
        '/apollo/perception/traffic_light',
        '/apollo/planning',
        '/apollo/prediction',
        '/apollo/routing_request',
        '/apollo/routing_response',
        '/apollo/localization/pose',
        '/apollo/drive_event',
        '/tf',
        '/tf_static',
        '/apollo/monitor',
        '/apollo/monitor/system_status',
        '/apollo/monitor/static_info',
    ]

    @classmethod
    def process_bag(cls, input_bag, output_bag):
        print("filtering: {} -> {}".format(input_bag, output_bag))
        output_dir = os.path.dirname(output_bag)
        if output_dir != "" and not os.path.exists(output_dir):
            os.makedirs(output_dir)
        try:
            with rosbag.Bag(input_bag, 'r') as bag_in:
                try:
                    with rosbag.Bag(output_bag, 'w') as bag_out:
                        for topic, msg, t in bag_in.read_messages(
                                topics=SamplePNC.TOPICS):
                            bag_out.write(topic, msg, t)
                except rosbag.ROSBagException as e:
                    print("Write file {} raised ROSBagException: {}".format(
                        input_bag, e))
                except ValueError as e:
                    print("Write file {} raised ValueError: {}".format(
                        input_bag, e))
                except:
                    print("Write file {} raised unknown exception".format(
                        output_bag))
        except rosbag.ROSBagException as e:
            print("open {} raised ROSBagException: {} ".format(input_bag, e))
        except rosbag.ROSBagFormatException as e:
            print("open {} raised ROSBagFormatException: {}".format(
                input_bag, e))
        except rosbag.ROSBagUnindexedException as e:
            print("open {} raised ROSBagUnindexedException: {} ".format(
                input_bag, e))
        except:
            print("Open {} failed with unknown exception".format(input_bag))


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="Filter pnc rosbag")
    parser.add_argument('input', type=str, help="the input rosbag")
    parser.add_argument('output', type=str, help="the output rosbag")
    args = parser.parse_args()
    SamplePNC.process_bag(args.input, args.output)
