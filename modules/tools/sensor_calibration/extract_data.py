#!/usr/bin/env python

###############################################################################
# Copyright 2019 The Apollo Authors. All Rights Reserved.
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
This is a tool to extract useful information from given record files. It does
self-check the validity of the uploaded data and able to inform developer's when
the data is not qualified, and reduce the size of uploaded data significantly.
"""

from datetime import datetime
import argparse
import os
import sys
import shutil
import six

import numpy as np
import cv2
import pypcd

from cyber_py.record import RecordReader
from cyber.proto import record_pb2

from modules.drivers.proto import sensor_image_pb2
from modules.drivers.proto import pointcloud_pb2
from modules.localization.proto import gps_pb2
from data_file_object import *
#from scripts.record_bag import SMALL_TOPICS


SMALL_TOPICS = [
    '/apollo/canbus/chassis',
    '/apollo/canbus/chassis_detail',
    '/apollo/control',
    '/apollo/control/pad',
    '/apollo/drive_event',
    '/apollo/guardian',
    '/apollo/hmi/status',
    '/apollo/localization/pose',
    '/apollo/localization/msf_gnss',
    '/apollo/localization/msf_lidar',
    '/apollo/localization/msf_status',
    '/apollo/monitor',
    '/apollo/monitor/system_status',
    '/apollo/navigation',
    '/apollo/perception/obstacles',
    '/apollo/perception/traffic_light',
    '/apollo/planning',
    '/apollo/prediction',
    '/apollo/relative_map',
    '/apollo/routing_request',
    '/apollo/routing_response',
    '/apollo/routing_response_history',
    '/apollo/sensor/conti_radar',
    '/apollo/sensor/delphi_esr',
    '/apollo/sensor/gnss/best_pose',
    '/apollo/sensor/gnss/corrected_imu',
    '/apollo/sensor/gnss/gnss_status',
    '/apollo/sensor/gnss/imu',
    '/apollo/sensor/gnss/ins_stat',
    '/apollo/sensor/gnss/odometry',
    '/apollo/sensor/gnss/raw_data',
    '/apollo/sensor/gnss/rtk_eph',
    '/apollo/sensor/gnss/rtk_obs',
    '/apollo/sensor/gnss/heading',
    '/apollo/sensor/mobileye',
    '/tf',
    '/tf_static',
]

CYBER_PATH = os.environ['CYBER_PATH']
CYBER_RECORD_HEADER_LENGTH = 2048

IMAGE_OBJ = sensor_image_pb2.Image()
POINTCLOUD_OBJ = pointcloud_pb2.PointCloud()
GPS_OBJ = gps_pb2.Gps()

def process_dir(path, operation):
    """Create or remove directory."""
    try:
        if operation == 'create':
            print('create folder: %s' % path)
            os.makedirs(path)
        elif operation == 'remove':
            os.remove(path)
        else:
            print('Error! Unsupported operation %s for directory.' % operation)
            return False
    except OSError as e:
        print('Failed to %s directory: %s. Error: %s' %
              (operation, path, six.text_type(e)))
        return False

    return True

def extract_camera_data(dest_dir, msg):
    """Extract camera file from message according to rate."""
    # TODO(gchen-Apollo): change saving logic
    cur_time_second = msg.timestamp
    image = IMAGE_OBJ
    image.ParseFromString(msg.message)
    # Save image according to cyber format, defined in sensor camera proto.
    # height = 4, image height, that is, number of rows.
    # width = 5,  image width, that is, number of columns.
    # encoding = 6, as string, type is 'rgb8', 'bgr8' or 'gray'.
    # step = 7, full row length in bytes.
    # data = 8, actual matrix data in bytes, size is (step * rows).
    # type = CV_8UC1 if image step is equal to width as gray, CV_8UC3
    # if step * 3 is equal to width.
    if image.encoding == 'rgb8' or image.encoding == 'bgr8':
        if image.step != image.width * 3:
            print('Image.step %d does not equal to Image.width %d * 3 for color image.'
                  % (image.step, image.width))
            return False
    elif image.encoding == 'gray' or image.encoding == 'y':
        if image.step != image.width:
            print('Image.step %d does not equal to Image.width %d or gray image.'
                  % (image.step, image.width))
            return False
    else:
        print('Unsupported image encoding type: %s.' % image.encoding)
        return False

    channel_num = image.step / image.width
    image_mat = np.fromstring(image.data, dtype=np.uint8).reshape(
        (image.height, image.width, channel_num))

    image_file = os.path.join(dest_dir, '{}.png'.format(cur_time_second))
    # Save image in BGR oder
    if image.encoding == 'rgb8':
        cv2.imwrite(image_file, cv2.cvtColor(image_mat, cv2.COLOR_RGB2BGR))
    else:
        cv2.imwrite(image_file, image_mat)

def convert_xyzit_pb_to_array(xyz_i_t, data_type):
    arr = np.zeros(len(xyz_i_t), dtype=data_type)
    for i, point in enumerate(xyz_i_t):
        arr[i] = (point.x, point.y, point.z,
                  point.intensity, point.timestamp)

    return arr

def make_xyzit_point_cloud(xyz_i_t):
    """ Make a pointcloud object from PointXYZIT message, as in Pointcloud.proto.
    message PointXYZIT {
      optional float x = 1 [default = nan];
      optional float y = 2 [default = nan];
      optional float z = 3 [default = nan];
      optional uint32 intensity = 4 [default = 0];
      optional uint64 timestamp = 5 [default = 0];
    }
    """

    md = {'version': .7,
          'fields': ['x', 'y', 'z', 'intensity', 'timestamp'],
          'count': [1, 1, 1, 1, 1],
          'width': len(xyz_i_t),
          'height': 1,
          'viewpoint': [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
          'points': len(xyz_i_t),
          'type': ['F', 'F', 'F', 'U', 'F'],
          'size': [4, 4, 4, 4, 8],
          'data': 'binary_compressed'}

    typenames = []
    for t, s in zip(md['type'], md['size']):
        np_type = pypcd.pcd_type_to_numpy_type[(t, s)]
        typenames.append(np_type)

    np_dtype = np.dtype(zip(md['fields'], typenames))
    pc_data = convert_xyzit_pb_to_array(xyz_i_t, data_type=np_dtype)
    pc = pypcd.PointCloud(md, pc_data)
    return pc

def extract_pcd_data(dest_dir, msg):
    """
    Transform protobuf PointXYZIT to standard PCL bin_compressed_file(*.pcd).
    """
    cur_time_second = msg.timestamp
    pointcloud = POINTCLOUD_OBJ
    pointcloud.ParseFromString(msg.message)

    pc_meta = make_xyzit_point_cloud(pointcloud.point)
    pcd_file = os.path.join(dest_dir, '{}.pcd'.format(cur_time_second))
    pypcd.save_point_cloud_bin_compressed(pc_meta, pcd_file)
    # TODO(gchen-Apollo): add saint check
    return True

def extract_gps_data(dest_dir, msg, out_msgs):
    """
    Save gps information to bin file, to be fed into following tools
    """
    if not isinstance(out_msgs, list):
        raise ValueError("Gps/Odometry msg should be saved as a list, not %s"
                         % type(out_msgs))

    gps = GPS_OBJ
    gps.ParseFromString(msg.message)

    # all double, except point_type is int32
    ts = gps.header.timestamp_sec
    point_type = 0
    qw = gps.localization.orientation.qw
    qx = gps.localization.orientation.qx
    qy = gps.localization.orientation.qy
    qz = gps.localization.orientation.qz
    x = gps.localization.position.x
    y = gps.localization.position.y
    z = gps.localization.position.z
    # save 9 values as a tuple, for eaisier struct packing during storage
    out_msgs.append((ts, point_type, qw, qx, qy, qz, x, y, z))

    print(gps)

    # TODO(gchen-Apollo): build class, to encapsulate inner data structure
    return out_msgs

def get_sensor_channel_list(record_file):
    """Get the channel list of sensors for calibration."""
    record_reader = RecordReader(record_file)
    return set(channel_name for channel_name in record_reader.get_channellist()
               if 'sensor' in channel_name)


def extract_channel_data(output_path, msg, channel_msgs=None):
    """Process channel messages."""
    channel_desc = msg.data_type
    if channel_desc == 'apollo.drivers.Image':
        extract_camera_data(output_path, msg)
    elif channel_desc == 'apollo.drivers.PointCloud':
        extract_pcd_data(output_path, msg)
    elif channel_desc == 'apollo.localization.Gps':
        channel_msgs[msg.topic] = extract_gps_data(output_path, msg,
                                                   channel_msgs[msg.topic])
    else:
        # TODO(LiuJie/gchen-Apollo): Handle binary data extraction.
        print('Not implemented!')

    return True, channel_msgs

def validate_channel_list(channels, dictionary):
    ret = True
    for channel in channels:
        if channel not in dictionary:
            print('ERROR: channel %s does not exist in record sensor channels'
                  % channel)
            ret = False

    return ret

def in_range(v, s, e):
    return True if v >= s and v <= e else False

def extract_data(record_files, output_path, channels,
                 start_timestamp, end_timestamp, extraction_rates):
    """
    Extract the desired channel messages if channel_list is specified.
    Otherwise extract all sensor calibration messages according to
    extraction rate, 10% by default.
    """
    # all records have identical sensor channels.
    sensor_channels = get_sensor_channel_list(record_files[0])

    if (len(channels) > 0 and not validate_channel_list(channels,
                                                        sensor_channels)):
        print('The input channel list is invalid.')
        return False

    # Extract all the sensor channels if channel_list is empty(no input arguments).
    print(sensor_channels)
    if len(channels) == 0:
        channels = sensor_channels

    # Declare logging variables
    process_channel_success_num = len(channels)
    process_channel_failure_num = 0
    process_msg_failure_num = 0

    channel_success = {}
    channel_occur_time = {}
    channel_output_path = {}
    channel_messages = {}
    for channel in channels:
        channel_success[channel] = True
        channel_occur_time[channel] = -1
        topic_name = channel.replace('/', '_')
        channel_output_path[channel] = os.path.join(output_path, topic_name)
        process_dir(channel_output_path[channel], operation='create')

        if channel in SMALL_TOPICS:
            channel_messages[channel] = list()

    for record_file in record_files:
        record_reader = RecordReader(record_file)
        for msg in record_reader.read_messages():
            if msg.topic in channels:
                # Only care about messages in certain time intervals
                msg_timestamp_sec = msg.timestamp / 1e9
                if not in_range(msg_timestamp_sec, start_timestamp, end_timestamp):
                    continue

                channel_occur_time[msg.topic] += 1
                # Extract the topic according to extraction_rate
                if channel_occur_time[msg.topic] % extraction_rates[msg.topic] != 0:
                    continue

                ret, _ = extract_channel_data(
                    channel_output_path[msg.topic], msg, channel_messages)

                # Calculate parsing statistics
                if not ret:
                    process_msg_failure_num += 1
                    if channel_success[msg.topic]:
                        channel_success[msg.topic] = False
                        process_channel_failure_num += 1
                        process_channel_success_num -= 1
                        print('Failed to extract data from channel: %s in record %s'
                              % (msg.topic, record_file))

    # traverse the dict, if any channel topic stored as a list
    # then save the list as a summary file, mostly binary file
    for channel, messages in channel_messages.items():
        save_msg_list_to_file(channel_output_path[channel], channel, messages)

    # Logging statics about channel extraction
    print('Extracted sensor channel number [%d] from record files: %s'
          % (len(channels), ' '.join(record_files)))
    print('Successfully processed [%d] channels, and [%d] was failed.'
          % (process_channel_success_num, process_channel_failure_num))
    if process_msg_failure_num > 0:
        print('Channel extraction failure number is [%d].' % process_msg_failure_num)

    return True

def save_msg_list_to_file(out_path, channel, messages):
    if 'odometry' in channel:
        # generate file objects for small topics I/O
        file_path = os.path.join(out_path, 'messages.bin')
        odometry_file_obj = OdometryFileObject(file_path)
        print(len(messages))
        odometry_file_obj.save_to_file(messages)
    else:
        raise ValueError("saving function for {} not implemented".format(channel))

def generate_compressed_file(input_path, input_name,
                             output_path, compressed_file='sensor_data'):
    """
    Compress data extraction directory as a single tar.gz archive
    """
    cwd_path = os.getcwd()
    os.chdir(input_path)
    shutil.make_archive(base_name=os.path.join(output_path, compressed_file),
                        format='gztar',
                        root_dir=input_path,
                        base_dir=input_name)
    os.chdir(cwd_path)

def generate_extraction_rate_dict(channels, large_topic_extraction_rate,
                                  small_topic_extraction_rate=1):
    """
    Default extraction rate for small topics is 1, which means no sampling
    """

    # Validate extration_rate, and set it as an integer.
    if large_topic_extraction_rate < 1.0 or small_topic_extraction_rate < 1.0:
        raise ValueError("Extraction rate must be a number no less than 1.")

    large_topic_extraction_rate = np.floor(large_topic_extraction_rate)
    small_topic_extraction_rate = np.floor(small_topic_extraction_rate)

    rates = {}
    for channel in channels:
        if channel in SMALL_TOPICS:
            rates[channel] = small_topic_extraction_rate
        else:
            rates[channel] = large_topic_extraction_rate

    return rates

def validate_record(record_file):
    """Validate the record file."""
    # Check the validity of a cyber record file according to header info.
    record_reader = RecordReader(record_file)
    header_msg = record_reader.get_headerstring()
    header = record_pb2.Header()
    header.ParseFromString(header_msg)
    print("header is {}".format(header))

    if not header.is_complete:
        print('Record file: %s is not completed.' % record_file)
        return False
    if header.size == 0:
        print('Record file: %s. size is 0.' % record_file)
        return False
    if header.major_version != 1 and header.minor_version != 0:
        print('Record file: %s. version [%d:%d] is wrong.' %
              (record_file, header.major_version, header.minor_version))
        return False
    if header.begin_time >= header.end_time:
        print('Record file: %s. begin time [%s] is equal or larger than '
              'end time [%s].' %
              (record_file, header.begin_time, header.end_time))
        return False

    if header.message_number < 1 or header.channel_number < 1:
        print('Record file: %s. [message:channel] number [%d:%d] is invalid.' %
              (record_file, header.message_number, header.channel_number))
        return False

    # There should be at least one sensor channel
    sensor_channels = get_sensor_channel_list(record_file)
    if len(sensor_channels) < 1:
        print('Record file: %s. cannot find sensor channels.' % record_file)
        return False

    return True

def validate_record_files(record_files, kword='.record.'):

    # load file list from directory if needs
    file_abs_paths = []
    if not isinstance(record_files, list):
        raise ValueError("Record files must be in a list")

    if len(record_files) == 1 and os.path.isdir(record_files[0]):
        print('Load cyber records from: %s' % record_files[0])
        for f in os.listdir(record_files[0]):
            if kword in f:
                file_abs_path = os.path.join(record_files[0], f)
                if validate_record(file_abs_path):
                    file_abs_paths.append(file_abs_path)
                else:
                    print('Invalid record file: %s' % file_abs_path)
    else:
        for f in record_files:
            if not os.path.isfile(f):
                raise ValueError("Input cyber record does not exist or not a regular file: %s" % f)

            if validate_record(f):
                file_abs_paths.append(f)
            else:
                print('Invalid record file: %s' % f)

    if len(file_abs_paths) < 1:
        raise ValueError("All the input record files are invalid")

    # Validate all record files have the same sensor topics
    first_record_file = file_abs_paths[0]
    default_sensor_channels = get_sensor_channle_list(first_record_file)
    for i, f in enumerate(file_abs_paths[1:]):
        sensor_channels = get_sensor_channel_list(f)
        if sensor_channels != default_sensor_channels:
            print('Default sensor channel list in %s is: ' % first_record_file)
            print(default_sensor_channels)
            print('but sensor channel list in %s is: ' % file_abs_paths[i])
            print(sensor_channels)
            raise ValueError("The record files should contain the same channel list")

    return file_abs_paths


def main():
    """
    Main function
    """
    if CYBER_PATH is None:
        print('Error: environment variable CYBER_PATH was not found, '
              'set environment first.')
        sys.exit(1)

    os.chdir(CYBER_PATH)

    parser = argparse.ArgumentParser(
        description='A tool to extract data information for sensor calibration.')
    parser.add_argument("-i", "--record_path", action="append", default=[], required=True,
                        dest='record_list',
                        help="Specify the record file to extract data information.")
    parser.add_argument("-o", "--output_path", action="store", type=str,
                        default="./extracted_data",
                        help="The output directory to restore message.")
    parser.add_argument("-z", "--compressed_file", action="store", type=str,
                        default="extraction_data", help="The output compressed filename.")
    parser.add_argument("-c", "--channel_name", dest='channel_list', action="append",
                        default=[], help="list of channel_name that needs parsing.")
    parser.add_argument("-s", "--start_timestamp", action="store", type=float,
                        default=np.finfo(np.float32).min,
                        help="Specify the begining time to extract data information.")
    parser.add_argument("-e", "--end_timestamp", action="store", type=float,
                        default=np.finfo(np.float32).max,
                        help="Specify the ending timestamp to extract data information.")
    parser.add_argument("-r", "--extraction_rate", action="store", type=int,
                        default=10, help="extraction rate for channel with large storage cost.")

    args = parser.parse_args()

    print('parsing the following channels: %s' % args.channel_list)


    valid_record_list = validate_record_files(args.record_list, kword='.record.')

    # Create directory to save the extracted data
    # use time now() as folder name
    output_relative_path = datetime.now().strftime("%Y-%m-%d-%H-%M")
    output_abs_path = os.path.join(args.output_path, output_relative_path)

    ret = process_dir(output_abs_path, 'create')
    if not ret:
        print('Failed to create extrated data directory: %s' % output_abs_path)
        sys.exit(1)

    channel_list = set(channel_name for channel_name in args.channel_list)
    extraction_rate_dict = generate_extraction_rate_dict(
        channel_list,
        large_topic_extraction_rate=args.extraction_rate,
        small_topic_extraction_rate=1)

    ret = extract_data(valid_record_list, output_abs_path, channel_list,
                       args.start_timestamp, args.end_timestamp, extraction_rate_dict)
    if not ret:
        print('Failed to extract data!')

    generate_compressed_file(input_path=args.output_path,
                             input_name=output_relative_path,
                             output_path=args.output_path,
                             compressed_file=args.compressed_file)

    print('Data extraction is completed successfully!')
    sys.exit(0)

if __name__ == '__main__':
    main()
