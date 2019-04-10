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
from google.protobuf import text_format

from cyber_py.record import RecordReader
from cyber.proto import record_pb2
from modules.tools.sensor_calibration.proto import extractor_config_pb2

from sensor_msg_extractor import *
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

def get_sensor_channel_list(record_file):
    """Get the channel list of sensors for calibration."""
    record_reader = RecordReader(record_file)
    return set(channel_name for channel_name in record_reader.get_channellist()
               if 'sensor' in channel_name)

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

def build_parser(channel, output_path):
    parser = None
    if channel.endswith("/image"):
        parser = ImageParser(output_path=output_path, instance_saving=True)
    elif channel.endswith("/PointCloud2"):
        parser = PointCloudParser(output_path=output_path, instance_saving=True)
    elif channel.endswith("/gnss/odometry"):
        parser = GpsParser(output_path=output_path, instance_saving=False)
    else:
        raise ValueError("Not Support this channel type: %s" %channel)

    return parser

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
    #channel_messages = {}
    channel_parsers = {}
    for channel in channels:
        channel_success[channel] = True
        channel_occur_time[channel] = -1
        topic_name = channel.replace('/', '_')
        channel_output_path[channel] = os.path.join(output_path, topic_name)
        process_dir(channel_output_path[channel], operation='create')
        channel_parsers[channel] =\
            build_parser(channel, channel_output_path[channel])

        # if channel in SMALL_TOPICS:
        #     channel_messages[channel] = list()

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

                ret = channel_parsers[msg.topic].parse_sensor_message(msg)

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
    for channel, parser in channel_parsers.items():
        save_combined_messages_info(parser, channel)

    # Logging statics about channel extraction
    print('Extracted sensor channel number [%d] from record files: %s'
          % (len(channels), ' '.join(record_files)))
    print('Successfully processed [%d] channels, and [%d] was failed.'
          % (process_channel_success_num, process_channel_failure_num))
    if process_msg_failure_num > 0:
        print('Channel extraction failure number is [%d].' % process_msg_failure_num)

    return True

def save_combined_messages_info(parser, channel):
    if not parser.save_messages_to_file():
        raise ValueError("cannot save combined messages into single file for : %s " % channel)


    if not parser.save_timestamps_to_file():
        raise ValueError("cannot save tiemstamp info for %s " % channel)

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
    default_sensor_channels = get_sensor_channel_list(first_record_file)
    for i, f in enumerate(file_abs_paths[1:]):
        sensor_channels = get_sensor_channel_list(f)
        if sensor_channels != default_sensor_channels:
            print('Default sensor channel list in %s is: ' % first_record_file)
            print(default_sensor_channels)
            print('but sensor channel list in %s is: ' % file_abs_paths[i])
            print(sensor_channels)
            raise ValueError("The record files should contain the same channel list")

    return file_abs_paths

def parse_channel_config(channels):
    channel_list = set()
    extraction_rate_dict = dict()

    for channel in channels:
        if channel.name in channel_list:
            raise ValueError("Duplicated channel config for : %s" % channel.name)
        else:
            channel_list.add(channel.name)
            extraction_rate_dict[channel.name] = channel.extraction_rate

    return channel_list, extraction_rate_dict

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
    # parser.add_argument("-i", "--record_path", action="append", default=[], required=True,
    #                     dest='record_list',
    #                     help="Specify the record file to extract data information.")
    # parser.add_argument("-o", "--output_path", action="store", type=str,
    #                     default="./extracted_data",
    #                     help="The output directory to restore message.")
    # # parser.add_argument("-z", "--compressed_file", action="store", type=str,
    # #                     default="extraction_data", help="The output compressed filename.")
    # parser.add_argument("-t", "--task_name", action="store", type=str, default="tmp",
    #                     help="name of the data extraction task, e.g., Camera_Lidar_Calibration.")
    # parser.add_argument("-c", "--channel_name", dest='channel_list', action="append",
    #                     default=[], help="list of channel_name that needs parsing.")
    # parser.add_argument("-s", "--start_timestamp", action="store", type=float,
    #                     default=np.finfo(np.float32).min,
    #                     help="Specify the begining time to extract data information.")
    # parser.add_argument("-e", "--end_timestamp", action="store", type=float,
    #                     default=np.finfo(np.float32).max,
    #                     help="Specify the ending timestamp to extract data information.")
    # parser.add_argument("-r", "--extraction_rate", action="store", type=int,
    #                     default=10, help="extraction rate for channel with large storage cost.")
    parser.add_argument("--config", action="store", type=str, required=True, dest="config",
                        help="protobuf text format configuration file abosolute path")
    args = parser.parse_args()

    config = extractor_config_pb2.DataExtractionConfig()
    with open(args.config, "r") as f:
        proto_block = f.read()
        text_format.Merge(proto_block, config)

    records = []
    for r in  config.records.record_path:
        records.append(str(r))

    valid_record_list = validate_record_files(records, kword='.record.')

    channels, extraction_rates = parse_channel_config(config.channels.channel)
    print('parsing the following channels: %s' % channels)

    start_timestamp = -1
    end_timestamp = -1
    if config.io_config.start_timestamp == "FLOAT_MIN":
        start_timestamp = np.finfo(np.float32).min
    else:
        start_timestamp = np.float32(config.io_config.start_timestamp)

    if config.io_config.end_timestamp == "FLOAT_MAX":
        end_timestamp = np.finfo(np.float32).max
    else:
        end_timestamp = np.float32(config.io_config.end_timestamp)

    # Create directory to save the extracted data
    # use time now() as folder name
    output_relative_path = config.io_config.task_name + datetime.now().strftime("-%Y-%m-%d-%H-%M")
    output_abs_path = os.path.join(config.io_config.output_path, output_relative_path)

    ret = process_dir(output_abs_path, 'create')
    if not ret:
        print('Failed to create extrated data directory: %s' % output_abs_path)
        sys.exit(1)

    ret = extract_data(valid_record_list, output_abs_path, channels,
                       start_timestamp, end_timestamp, extraction_rates)
    if not ret:
        print('Failed to extract data!')

    generate_compressed_file(input_path=config.io_config.output_path,
                             input_name=output_relative_path,
                             output_path=config.io_config.output_path,
                             compressed_file=config.io_config.task_name)

    print('Data extraction is completed successfully!')
    sys.exit(0)

if __name__ == '__main__':
    main()
