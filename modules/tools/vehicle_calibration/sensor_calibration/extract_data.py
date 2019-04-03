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

import os
import sys
import argparse
from datetime import datetime
import re
import tarfile

import six
import numpy as np
import cv2

from cyber_py.record import RecordReader
from cyber.proto import record_pb2

from modules.drivers.proto import sensor_image_pb2

CYBER_PATH = os.environ['CYBER_PATH']

CYBER_RECORD_HEADER_LENGTH = 2048
IMAGE_OBJ = sensor_image_pb2.Image()

def process_dir(path, operation):
    """
    Create or remove directory
    """
    try:
        if operation == 'create':
            print("create folder %s" % path)
            os.makedirs(path)
        elif operation == 'remove':
            os.remove(path)
        else:
            print('Error! Unsupported operation %s for directory.' % operation)
            return False
    except (OSError, IOError) as e:
        print('Failed to %s directory: %s. Error: %s' %
              (operation, path, six.text_type(e)))
        return False

    return True

def extract_camera_data(dest_dir, msg):
    """
    Extract camera file from message according to ratio
    """
    #TODO: change saving logic 
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
    if image.encoding == "rgb8" or image.encoding == "bgr8":
        if image.step != image.width * 3:
            print("Image.step %d does not equal Image.width %d * 3 for color image" %
                (image.step, image.width))
            return False
    elif image.encoding == "gray" or image.encoding == "y":
        if image.step != image.width:
            print("Image.step %d does not equal Image.width %d or gray image" %
                (image.step, image.width))
            return False
    else:
        print("Unsupported image encoding type %s" %encoding)
        return False

    channel_num = image.step / image.width
    image_mat = np.fromstring(image.data, dtype=np.uint8).reshape(
                    (image.height, image.width, channel_num))

    image_file = os.path.join(dest_dir, '{}.png'.format(cur_time_second))
    #python cv2 save image in BGR oder
    if image.encoding == "rgb8":
        cv2.imwrite(image_file, cv2.cvtColor(image_mat, cv2.COLOR_RGB2BGR))
    else:
        cv2.imwrite(image_file, image_mat)

def extract_pcd_data(dest_dir, msg, ratio):
    """
    Extract PCD file
    """
    time_nseconds = []
    pre_time_second = 0

    seq = 0
    # Check timestamp.
    while True:
        cur_time_second = msg.header.stamp.to_sec()
        if cur_time_second - pre_time_second < ratio:
            continue
        pre_time_second = cur_time_second
        time_nseconds.append(msg.header.stamp.to_nsec())
        # pcd = pcl.PointCloud()
        pcd = []
        pcd.from_file(msg)
        dfilter = pcd.make_statistical_outlier_filter()
        dfilter.set_mean_k(50)
        dfilter.set_std_dev_mul_thresh(1.0)
        pcd_file = os.path.join(dest_dir, '{}.pcd'.format(seq))
        dfilter.filter().to_file(pcd_file)
        seq += 1


def get_sensor_channel_list(record_file):
    """
    Get the channel list of sensors for calibration
    """
    record_reader = RecordReader(record_file)
    return set(channel_name for channel_name in record_reader.get_channellist()
            if 'sensor' in channel_name)
    # return [channel_name for channel_name in record_reader.get_channellist()
            # if 'sensor' in channel_name]


def validate_record(record_file):
    """
    Validate the record file
    """
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

    # There should be at least has one sensor channel
    sensor_channels = get_sensor_channel_list(record_file)
    if len(sensor_channels) < 1:
        print('Record file: %s. cannot find sensor channels.' % record_file)
        return False

    return True


def extract_channel_data(output_path, msg):
    """
    Process channel messages.
    """
    #    timestamp = msg.timestamp / float(1e9)
    #    if abs(begin_time - timestamp) > 2:
    channel_desc = msg.data_type
    if channel_desc == 'apollo.drivers.Image':
        extract_camera_data(output_path, msg)
    elif channel_desc == 'apollo.drivers.PointCloud':
        extract_pcd_data(output_path, msg, extraction_ratio)
    else:
        # (TODO) (Liujie/Yuanfan) Handle binary data extraction.
        print('Not implemented!')

    return True

def validate_channel_list(channels, dictionary):
    ret = True
    for channel in channels:
        if channel not in dictionary:
            print("ERROR: channel %s does not exist in record\
                    sensor channels" % channel)
            ret = False

    return ret

def in_range(v, s, e):
    return True if v >= s and v <= e else False

def extract_data(record_file, output_path, channel_list,
                start_timestamp, end_timestamp, extraction_ratio):
    """
    Extract the desired channel messages if channel_list is specified.
    Otherwise extract all sensor calibration messages according to
    extraction ratio, 10% by default.
    """
    #validate extration_ratio, and set it as an integer
    if extraction_ratio < 1.0:
        raise ValueError("Extraction rate must be a number no less than 1")
    extraction_ratio = np.floor(extraction_ratio)

    sensor_channels = get_sensor_channel_list(record_file)
    if len(channel_list) > 0 and validate_channel_list(
         channel_list, sensor_channels) is False:
        print("input channel list not valid")
        return False

    #If channel_list is empty(no input arguments), extract all the sensor channels
    print(sensor_channels)
    if len(channel_list) == 0:
        channel_list = sensor_channels

    #Declare logging variables
    process_channel_success_num = len(channel_list)
    process_channel_failure_num = 0
    process_msg_failure_num = 0

    channel_success_dict = {}
    channel_occur_time = {}
    channel_output_path = {}
    for channel in channel_list:
        channel_success_dict[channel] = True
        channel_occur_time[channel] = -1
        topic_name = channel.replace("/", "_")
        channel_output_path[channel] = os.path.join(output_path,topic_name)
        process_dir(channel_output_path[channel], operation="create")

    record_reader = RecordReader(record_file)
    for msg in record_reader.read_messages():
        if msg.topic in channel_list:
            # only care about messages in certain time intervals
            msg_timestamp_sec = msg.timestamp/1e9
            if not in_range(msg_timestamp_sec, start_timestamp, end_timestamp):
                continue

            channel_occur_time[msg.topic] += 1
            # extract the topic according to extraction_ratio
            if channel_occur_time[msg.topic] % extraction_ratio != 0:
                continue

            ret = extract_channel_data(channel_output_path[msg.topic], msg)
            # calculate parsing statistics
            if ret is False:
                process_msg_failure_num += 1
                if channel_success_dict[msg.topic] is True:
                    channel_success_dict[msg.topic] = False;
                    process_channel_failure_num += 1
                    process_channel_success_num -= 1
                    print('Failed to extract data from channel: %s' % msg.topic)

    #Logging statics about channel extraction
    print('Extracted sensor channel number [%d] in record file: %s' %
          (len(channel_list), record_file))
    print('Successfully processed [%d] channels, and [%d] was failed.' %
          (process_channel_success_num, process_channel_failure_num))
    if process_msg_failure_num > 0:
        print('Channel Extraction Failure number is: %d' % process_msg_failure_num)

    return True

def generate_compressed_file(extracted_data_path,
                             compressed_file='sensor_data'):
    """
    Compress each data file to compressed package
    """
    # Note that ZipFile support compress a directory in Python, but GZIP not.
    # We only support GZ compression type with tarfile for now.
    with tarfile.open(compressed_file + '.tar.gz', 'w:gz') as g_out:
        g_out.add(extracted_data_path, arcname=compressed_file)


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
    parser.add_argument("-i", "--record_path", action="store", type=str,
                        required=True,
                        help="Specify the record file to extract data information.")
    parser.add_argument("-o", "--output_path", action="store", type=str,
                        default="./extracted_data",
                        help="The output directory to restore message.")
    parser.add_argument("-z", "--compressed_file", action="store", type=str,
                        default="", help="The output directory to restore message.")
    parser.add_argument("-c", "--channel_name", dest='channel_list', action="append",
                        default=[], help="list of channel_name that needs parsing.")
    parser.add_argument("-s", "--start_timestamp", action="store", type=float,
                        default=np.finfo(np.float32).min, help="Specify the begining time to extract data information.")
    parser.add_argument("-e", "--end_timestamp", action="store", type=float,
                        default=np.finfo(np.float32).max, help="Specify the ending timestamp to extract data information.")
    parser.add_argument("-r", "--extraction_ratio", action="store", type=int,
                        default=10, help="The output compressed file.")

    args = parser.parse_args()

    print("parsing the following channels:%s" % args.channel_list)

    # (TODO: Liujie) Add logger info to trace the extraction process
    ret = validate_record(args.record_path)
    if ret is False:
        print('Failed to validate record file: %s' % args.record_path)
        sys.exit(1)

    # Create directory to save the extracted data
    output_path = args.output_path + re.sub(r'[^0-9]', '', str(datetime.now()))

    ret = process_dir(output_path, 'create')
    if ret is False:
        print('Failed to create extrated data directory: %s' % args.output_path)
        sys.exit(1)

    channel_list = set(channel_name for channel_name in args.channel_list)
    ret = extract_data(args.record_path, output_path, channel_list,
                       args.start_timestamp, args.end_timestamp, args.extraction_ratio)
    if ret is False:
        print('Failed to extract data!')

    generate_compressed_file(output_path, args.compressed_file)

    print('Data extraction is completed successfully!')
    sys.exit(0)

if __name__ == '__main__':
    main()
