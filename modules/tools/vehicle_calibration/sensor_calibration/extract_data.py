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
import cv2.cv as cv

from cyber_py.record import RecordReader
from cyber.proto import record_pb2

from modules.drivers.proto import sensor_image_pb2

CYBER_PATH = os.environ['CYBER_PATH']

CYBER_RECORD_HEADER_LENGTH = 2048

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

def extract_camera_data(dest_dir, msg, ratio):
    """
    Extract camera file from message according to ratio
    """
    time_nseconds = []
    pre_time_second = 0

    seq = 0
    # Check timestamp.
    #TODO: change saving logic 
    #while True:
    if True:
        cur_time_second = msg.timestamp

        image = sensor_image_pb2.Image()
        image.ParseFromString(msg.message)
        # Save image according to cyber format.
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
            image_mat = image_mat[:, :, ::-1]
            cv.SaveImage(image_file, cv.fromarray(image_mat))
        else:
            cv.SaveImage(image_file, cv.fromarray(image_mat))
        seq += 1

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
    return [channel_name for channel_name in record_reader.get_channellist()
            if 'sensor' in channel_name]


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


def extract_channel_data(record_reader, output_path, channel_name,
                         begin_time, extraction_ratio):
    """
    Process channel messages.
    """
    count = 0
    for msg in record_reader.read_messages():
        if count == 1: # test for now
            break
        if msg.topic != channel_name:
            continue
        count += 1
        timestamp = msg.timestamp / float(1e9)
        # The begin time to extract data should be at least later 1 second
        # than message time.
        if abs(begin_time - timestamp) > 2:
            channel_desc = msg.data_type
            if channel_desc == 'apollo.drivers.Image':
                extract_camera_data(output_path, msg, extraction_ratio)
            elif channel_desc == 'apollo.drivers.PointCloud':
                extract_pcd_data(output_path, msg, extraction_ratio)
            else:
                # (TODO) (Liujie/Yuanfan) Handle binary data extraction.
                print('Not implemented!')

    return True


def extract_data(record_file, output_path, channel_name, timestamp,
                 extraction_ratio):
    """
    Extract the desired channel messages if channel_name is specified.
    Otherwise extract all sensor calibration messages according to
    extraction ratio, 10% by default.
    """
    record_reader = RecordReader(record_file)
    if channel_name != ' ':
        ret = extract_channel_data(record_reader, output_path, channel_name,
                                   timestamp, extraction_ratio)
        return ret

    sensor_channels = get_sensor_channel_list(record_file)
    channel_num = len(sensor_channels)
    print('Sensor channel number [%d] in record file: %s' %
          (channel_num, record_file))

    process_channel_success_num = 0
    process_channel_failure_num = 0
    for msg in record_reader.read_messages():
        if msg.topic in sensor_channels:
            ret = extract_channel_data(record_reader, output_path, msg.topic,
                                       timestamp, extraction_ratio)
            if ret is False:
                print('Failed to extract data from channel: %s' % msg.topic)
                process_channel_failure_num += 1
                continue
            process_channel_success_num += 1

    print('Successfully processed [%d] channels, and [%d] was failed.' %
          process_channel_success_num, process_channel_failure_num)

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
    parser.add_argument("-r", "--record_path", action="store", type=str,
                        required=True,
                        help="Specify the record file to extract data information.")
    parser.add_argument("-o", "--output_path", action="store", type=str,
                        default="./extracted_data",
                        help="The output directory to restore message.")
    parser.add_argument("-z", "--compressed_file", action="store", type=str,
                        default="", help="The output directory to restore message.")
    parser.add_argument("-c", "--channel_name", action="store", type=str,
                        default="", help="The output compressed file.")
    parser.add_argument("-t", "--timestamp", action="store", type=float,
                        help="Specify the timestamp to extract data information.")
    parser.add_argument("-e", "--extraction_ratio", action="store", type=int,
                        default=10, help="The output compressed file.")

    args = parser.parse_args()

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

    ret = extract_data(args.record_path, output_path, args.channel_name,
                       args.timestamp, args.extraction_ratio)
    if ret is False:
        print('Failed to extract data!')

    generate_compressed_file(output_path, args.compressed_file)

    print('Data extraction is completed successfully!')
    sys.exit(0)

if __name__ == '__main__':
    main()
