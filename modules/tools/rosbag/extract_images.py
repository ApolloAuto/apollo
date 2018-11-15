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
Extract images from a recorded bag.

Usage:
    extract_images.py --input_bag=a.bag

See the gflags for more optional args.
"""

import os
import sys

import cv2
import cv_bridge
import gflags
import glog
import rosbag
import yaml

# Requried flags.
gflags.DEFINE_string('input_bag', None, 'Input bag path.')

# Optional flags.
gflags.DEFINE_string('output_path', './', 'Output path.')
gflags.DEFINE_string('weather', 'CLEAR', 'Options: CLEAR, SUNNY, RAINY.')
gflags.DEFINE_string('scene', 'CITY', 'Options: CITY, HIGHWAY.')
gflags.DEFINE_string('time_interval', 'DAYTIME', 'Options: DAYTIME, NIGHT.')
gflags.DEFINE_float('extract_rate', 3, 'Rate to extract image, in seconds.')

# Stable flags which rarely change.
gflags.DEFINE_string('topic', '/apollo/sensor/camera/obstacle/front_6mm',
                     'Source topic.')
gflags.DEFINE_integer('sensor_id', 436, 'Source sensor ID.')
gflags.DEFINE_string('capture_place', 'Multiple', 'E.g.: Multiple, Sunnyvale.')


def extract_meta_info(bag):
    """Extract information from a bag file, return an info dict."""
    # Extract from bag info.
    info_dict = yaml.load(bag._get_yaml_info())
    meta_info = {
        'car_id': 'MKZ056',
        'driver': 'UNKNOWN',
        'start': int(info_dict['start']),
        'end': int(info_dict['end']),
    }

    # Extract from bag message.
    kStaticInfoTopic = '/apollo/monitor/static_info'
    static_info = next(
        (msg for _, msg, _ in bag.read_messages(topics=[kStaticInfoTopic])),
        None)
    if static_info is not None:
        if static_info.vehicle.name:
            meta_info['car_id'] = static_info.vehicle.name.upper()
        if static_info.user.driver:
            meta_info['driver'] = static_info.user.driver
    return meta_info


def extract_images(bag, dst_dir, args):
    """Extract images to the destination dir."""
    time_nsecs = []
    pre_time_sec = 0
    bridge = cv_bridge.CvBridge()
    seq = 0
    for _, msg, t in bag.read_messages(topics=args.topic):
        # Check timestamp.
        cur_time_sec = msg.header.stamp.to_sec()
        if cur_time_sec - pre_time_sec < args.extract_rate:
            continue
        pre_time_sec = cur_time_sec
        time_nsecs.append(msg.header.stamp.to_nsec())

        # Save image.
        seq += 1
        msg.encoding = 'yuv422'
        img = bridge.imgmsg_to_cv2(msg, 'yuv422')
        img = cv2.cvtColor(img, cv2.COLOR_YUV2BGR_YUYV)
        img_file = os.path.join(dst_dir, '{}.jpg'.format(seq))
        cv2.imwrite(img_file, img)

        glog.info('#{}: header.seq={}, header.stamp={}, saved as {}'.format(
            seq, msg.header.seq, cur_time_sec, img_file))
    return time_nsecs


def process_bag(bag, args):
    """Process a bag."""
    meta_info = extract_meta_info(bag)
    dst_dir_name = '{}_{}_{}_{}'.format(meta_info['car_id'], args.sensor_id,
                                        meta_info['start'], meta_info['end'])
    dst_dir = os.path.join(args.output_path, dst_dir_name)
    if not os.path.exists(dst_dir):
        os.makedirs(dst_dir)

    # Generate meta file.
    meta_file = os.path.join(dst_dir, dst_dir_name + '.meta')
    with open(meta_file, 'w') as meta_w:
        meta_w.write('car_id:{}\n'.format(meta_info['car_id']))
        meta_w.write('driver:{}\n'.format(meta_info['driver']))
        meta_w.write('capture_place:{}\n'.format(args.capture_place))
        meta_w.write('weather:{}\n'.format(args.weather))
        meta_w.write('topic:{}\n'.format(args.topic))
        meta_w.write('scene:{}\n'.format(args.scene))
        meta_w.write('time_interval:{}\n'.format(args.time_interval))

    # Generate images.
    time_nsecs = extract_images(bag, dst_dir, args)

    # Generate timestamp sequence.
    timestamp_file = os.path.join(dst_dir, 'timestamp.txt')
    with open(timestamp_file, 'w') as timestamp_w:
        timestamp_w.write('seq\ttimestamp_ns\n')
        for seq, timestamp_ns in enumerate(time_nsecs, start=1):
            timestamp_w.write('{}\t{}\n'.format(seq, timestamp_ns))


def main():
    """Entry point."""
    gflags.FLAGS(sys.argv)
    with rosbag.Bag(gflags.FLAGS.input_bag) as bag:
        process_bag(bag, gflags.FLAGS)

if __name__ == '__main__':
    main()
