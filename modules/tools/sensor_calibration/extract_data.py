#!/usr/bin/env python3

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
import math
import os
import shutil
import six
import sys
import time
import yaml

from absl import app
from absl import flags
from google.protobuf import text_format
import numpy as np

from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3.record import RecordReader
from cyber.proto import record_pb2
from modules.dreamview.proto import preprocess_table_pb2
from modules.tools.common.proto_utils import get_pb_from_text_file
from modules.tools.sensor_calibration.configuration_yaml_generator import ConfigYaml
from modules.tools.sensor_calibration.extract_static_data import get_subfolder_list, select_static_image_pcd
from modules.tools.sensor_calibration.proto import extractor_config_pb2
from modules.tools.sensor_calibration.sanity_check import sanity_check
from modules.tools.sensor_calibration.sensor_msg_extractor import GpsParser, ImageParser, PointCloudParser, PoseParser, ContiRadarParser

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

flags.DEFINE_string('config', '',
                    'protobuf text format configuration file abosolute path')
flags.DEFINE_string('root_dir', '/apollo/modules/tools/sensor_calibration',
                    'program root dir')

FLAGS = flags.FLAGS
POINTCLOUD_LOCALIZATION_TS_DIFF = 2 * 1000 * 1000 * 1000

class Extractor(object):
    def __init__(self):
        self.node = cyber.Node("sensor_calibration_preprocessor")
        self.writer = self.node.create_writer("/apollo/dreamview/progress",
                                              preprocess_table_pb2.Progress, 6)
        self.config = extractor_config_pb2.DataExtractionConfig()
        self.progress = preprocess_table_pb2.Progress()
        self.progress.percentage = 0.0
        self.progress.log_string = "Preprocessing in progress..."
        self.progress.status = preprocess_table_pb2.Status.UNKNOWN
        self.has_check_xyzit_legality = False
        # first localization pose timestamp before first pointcloud 
        self.first_localization_ts = None
        # first pointcloud timestamp from all sensor lidar channels
        self.first_pointcloud_ts = dict()
        try:
            get_pb_from_text_file(FLAGS.config, self.config)
        except text_format.ParseError:
            print(f'Error: Cannot parse {FLAGS.config} as text proto')
        self.records = []
        for r in self.config.records.record_path:
            self.records.append(str(r))
        self.start_timestamp = -1
        self.end_timestamp = -1
        if self.config.io_config.start_timestamp == "FLOAT_MIN":
            self.start_timestamp = np.finfo(np.float32).min
        else:
            self.start_timestamp = np.float32(
                self.config.io_config.start_timestamp)

        if self.config.io_config.end_timestamp == "FLOAT_MAX":
            self.end_timestamp = np.finfo(np.float32).max
        else:
            self.end_timestamp = np.float32(
                self.config.io_config.end_timestamp)

    @staticmethod
    def process_dir(path, operation):
        """Create or remove directory."""
        try:
            if operation == 'create':
                if os.path.exists(path):
                    print(f'folder: {path} exists')
                else:
                    print(f'create folder: {path}')
                    os.makedirs(path)
            elif operation == 'remove':
                os.remove(path)
            else:
                print(
                    f'Error! Unsupported operation {operation} for directory.')
                return False
        except OSError as e:
            print(f'Failed to {operation} directory: {path}. '
                  f'Error: {six.text_type(e)}')
            return False

        return True

    @staticmethod
    def get_sensor_channel_list(record_file):
        """Get the channel list of sensors for calibration."""
        record_reader = RecordReader(record_file)
        return set(channel_name
                   for channel_name in record_reader.get_channellist()
                   if 'sensor' in channel_name
                   or '/localization/pose' in channel_name)

    @staticmethod
    def validate_channel_list(channels, dictionary):
        ret = True
        for channel in channels:
            if channel not in dictionary:
                print(f'ERROR: channel {channel} does not exist in '
                      'record sensor channels')
                ret = False
        return ret

    @staticmethod
    def in_range(v, s, e):
        return True if v >= s and v <= e else False

    @staticmethod
    def build_parser(channel, output_path):
        parser = None
        if channel.endswith("/image"):
            parser = ImageParser(output_path=output_path, instance_saving=True)
        elif channel.endswith("/PointCloud2"):
            parser = PointCloudParser(output_path=output_path,
                                      instance_saving=True)
        elif channel.endswith("/gnss/odometry"):
            parser = GpsParser(output_path=output_path, instance_saving=False)
        elif channel.endswith("/localization/pose"):
            parser = PoseParser(output_path=output_path, instance_saving=False)
        elif channel.startswith("/apollo/sensor/radar"):
            parser = ContiRadarParser(output_path=output_path,
                                      instance_saving=True)
        else:
            raise ValueError(f"Not Support this channel type: {channel}")
        return parser

    def print_and_publish(self,
                          str,
                          status=preprocess_table_pb2.Status.UNKNOWN):
        """status: 0 for success, 1 for fail, 2 for unknown"""
        print(str)
        self.progress.log_string = str
        self.progress.status = status
        self.writer.write(self.progress)
        time.sleep(0.5)

    def check_xyzit_legality(self):
        """
        check point cloud data legality.
        xyzit is a point cloud message: x, y, z, intensity, timestamp. 
        """
        # check point cloud data timestamp legality.
        self.has_check_xyzit_legality = True
        for msg_topic, first_pointcloud_ts in self.first_pointcloud_ts.items():
            pointcloud_localization_ts_diff = math.fabs(first_pointcloud_ts - self.first_localization_ts)
            if pointcloud_localization_ts_diff > POINTCLOUD_LOCALIZATION_TS_DIFF:
                log_str = (f"The pointcloud data contains illegal data in channel:{msg_topic},"
                f"the difference between pointcloud data timestamp and localization data timestamp is:{pointcloud_localization_ts_diff}")
                self.print_and_publish(
                            log_str, preprocess_table_pb2.Status.FAIL)
                raise ValueError(
                f"The pointcloud data contains illegal data in channel:{msg_topic},"
                f"the difference between pointcloud data timestamp and localization data timestamp is:{pointcloud_localization_ts_diff}")

    def extract_data(self, record_files, output_path, channels,
                     extraction_rates):
        """
        Extract the desired channel messages if channel_list is specified.
        Otherwise extract all sensor calibration messages according to
        extraction rate, 10% by default.
        """
        # all records have identical sensor channels.
        sensor_channels = self.get_sensor_channel_list(record_files[0])

        if (len(channels) > 0
                and not self.validate_channel_list(channels, sensor_channels)):
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
        # channel_messages = {}
        channel_parsers = {}
        channel_message_number = {}
        channel_processed_msg_num = {}
        lidar_sensor_channels_num = 0
        
        for channel in channels:
            channel_success[channel] = True
            channel_occur_time[channel] = -1
            topic_name = channel.replace('/', '_')
            channel_output_path[channel] = os.path.join(
                output_path, topic_name)
            self.process_dir(channel_output_path[channel], operation='create')
            channel_parsers[channel] =\
                self.build_parser(channel, channel_output_path[channel])
            channel_message_number[channel] = 0
            for record_file in record_files:
                record_reader = RecordReader(record_file)
                channel_message_number[
                    channel] += record_reader.get_messagenumber(channel)
            channel_message_number[channel] = channel_message_number[
                channel] // extraction_rates[channel]
            if "/PointCloud2" in channel:
                lidar_sensor_channels_num += 1

        channel_message_number_total = 0
        for num in channel_message_number.values():
            channel_message_number_total += num
        channel_processed_msg_num = 0
        
        # if channel in SMALL_TOPICS:
        # channel_messages[channel] = list()
        for record_file in record_files:
            record_reader = RecordReader(record_file)
            for msg in record_reader.read_messages():
                if msg.topic in channels:
                    # Only care about messages in certain time intervals
                    msg_timestamp_sec = msg.timestamp / 1e9
                    if not self.in_range(msg_timestamp_sec,
                                         self.start_timestamp,
                                         self.end_timestamp):
                        continue

                    channel_occur_time[msg.topic] += 1
                    # Extract the topic according to extraction_rate
                    if channel_occur_time[msg.topic] % extraction_rates[
                            msg.topic] != 0:
                        continue
                    # check pointcloud data legality
                    if not self.has_check_xyzit_legality:
                        if "/localization" in msg.topic:
                            self.first_localization_ts = channel_parsers[msg.topic].get_timestamps(msg) 
                        if "/PointCloud2" in msg.topic:
                            self.first_pointcloud_ts[msg.topic] = channel_parsers[msg.topic].get_timestamps(msg)
                            if len(self.first_pointcloud_ts) == lidar_sensor_channels_num and self.first_localization_ts is not None:
                                self.check_xyzit_legality()
                
                    ret = channel_parsers[msg.topic].parse_sensor_message(msg)
                    channel_processed_msg_num += 1
                    self.progress.percentage = channel_processed_msg_num / \
                        channel_message_number_total * 90.0
                    # Calculate parsing statistics
                    if not ret:
                        process_msg_failure_num += 1
                        if channel_success[msg.topic]:
                            channel_success[msg.topic] = False
                            process_channel_failure_num += 1
                            process_channel_success_num -= 1
                            log_string = (
                                'Failed to extract data from channel: '
                                f'{msg.topic} in record {record_file}')
                            print(log_string)
                            self.progress.log_string = log_string

                    self.writer.write(self.progress)

        # traverse the dict, if any channel topic stored as a list
        # then save the list as a summary file, mostly binary file
        for channel, parser in channel_parsers.items():
            self.save_combined_messages_info(parser, channel)

        # Logging statics about channel extraction
        self.print_and_publish(
            (f"Extracted sensor channel number {len(channels)} "
             f"from record files: {' '.join(record_files)}"))
        self.print_and_publish(
            (f'Successfully processed {process_channel_success_num} channels, '
             f'and {process_channel_failure_num} was failed.'))
        if process_msg_failure_num > 0:
            self.print_and_publish(
                f'Channel extraction failure number is {process_msg_failure_num}.',
                preprocess_table_pb2.Status.FAIL)

        return True

    @staticmethod
    def save_combined_messages_info(parser, channel):
        if not parser.save_messages_to_file():
            raise ValueError(
                f"cannot save combined messages into single file for : {channel}"
            )
        if not parser.save_timestamps_to_file():
            raise ValueError(f"cannot save tiemstamp info for {channel}")

    @staticmethod
    def generate_compressed_file(input_path,
                                 input_name,
                                 output_path,
                                 compressed_file='sensor_data'):
        """
        Compress data extraction directory as a single tar.gz archive
        """
        cwd_path = os.getcwd()
        os.chdir(input_path)
        shutil.make_archive(base_name=os.path.join(output_path,
                                                   compressed_file),
                            format='gztar',
                            root_dir=input_path,
                            base_dir=input_name)
        os.chdir(cwd_path)

    @staticmethod
    def generate_extraction_rate_dict(channels,
                                      large_topic_extraction_rate,
                                      small_topic_extraction_rate=1):
        """
        Default extraction rate for small topics is 1, which means no sampling
        """

        # Validate extration_rate, and set it as an integer.
        if large_topic_extraction_rate < 1.0 or small_topic_extraction_rate < 1.0:
            raise ValueError(
                "Extraction rate must be a number no less than 1.")

        large_topic_extraction_rate = np.floor(large_topic_extraction_rate)
        small_topic_extraction_rate = np.floor(small_topic_extraction_rate)

        rates = {}
        for channel in channels:
            if channel in SMALL_TOPICS:
                rates[channel] = small_topic_extraction_rate
            else:
                rates[channel] = large_topic_extraction_rate

        return rates

    @staticmethod
    def validate_record(record_file):
        """Validate the record file."""
        # Check the validity of a cyber record file according to header info.
        record_reader = RecordReader(record_file)
        header_msg = record_reader.get_headerstring()
        header = record_pb2.Header()
        header.ParseFromString(header_msg)
        print(f"header is {header}")

        if not header.is_complete:
            print(f'Record file: {record_file} is not completed.')
            return False
        if header.size == 0:
            print(f'Record file: {record_file}. size is 0.')
            return False
        if header.major_version != 1 and header.minor_version != 0:
            print(
                f'Record file: {record_file}. version [{header.major_version}: '
                f'{header.minor_version}] is wrong.')
            return False
        if header.begin_time >= header.end_time:
            print(
                f'Record file: {record_file}. begin time [{header.begin_time}] '
                f'is equal or larger than end time [{header.end_time}].')
            return False

        if header.message_number < 1 or header.channel_number < 1:
            print(
                f'Record file: {record_file}. [message:channel] number '
                f'[{header.message_number}:{header.channel_number}] is invalid.'
            )
            return False

        # There should be at least one sensor channel
        sensor_channels = Extractor.get_sensor_channel_list(record_file)
        if len(sensor_channels) < 1:
            print(f'Record file: {record_file}. cannot find sensor channels.')
            return False

        return True

    def validate_record_files(self, kword='.record.'):
        # load file list from directory if needs
        file_abs_paths = []
        if not isinstance(self.records, list):
            raise ValueError("Record files must be in a list")

        records = self.records
        if len(records) == 1 and os.path.isdir(records[0]):
            print(f'Load cyber records from: {records[0]}')
            for f in sorted(os.listdir(records[0])):
                if kword in f:
                    file_abs_path = os.path.join(records[0], f)
                    if Extractor.validate_record(file_abs_path):
                        file_abs_paths.append(file_abs_path)
                    else:
                        print(f'Invalid record file: {file_abs_path}')
        else:
            for f in records:
                if not os.path.isfile(f):
                    raise ValueError("Input cyber record does not exist "
                                     f"or not a regular file: {f}")

                if Extractor.validate_record(f):
                    file_abs_paths.append(f)
                else:
                    print(f'Invalid record file: {f}')

        if len(file_abs_paths) < 1:
            raise ValueError("All the input record files are invalid")

        # Validate all record files have the same sensor topics
        first_record_file = file_abs_paths[0]
        default_sensor_channels = Extractor.get_sensor_channel_list(
            first_record_file)
        for i, f in enumerate(file_abs_paths[1:]):
            sensor_channels = Extractor.get_sensor_channel_list(f)
            if sensor_channels != default_sensor_channels:
                print(
                    f'Default sensor channel list in {first_record_file} is: ')
                print(default_sensor_channels)
                print(f'but sensor channel list in {file_abs_paths[i]} is: ')
                print(sensor_channels)
                raise ValueError(
                    "The record files should contain the same channel list")

        return file_abs_paths

    def parse_channel_config(self):
        channel_list = set()
        extraction_rate_dict = dict()

        for channel in self.config.channels.channel:
            if channel.name in channel_list:
                raise ValueError(
                    f"Duplicated channel config for : {channel.name}")
            else:
                channel_list.add(channel.name)
                extraction_rate_dict[channel.name] = channel.extraction_rate
        return channel_list, extraction_rate_dict

    @staticmethod
    def get_substring(str, prefix, suffix):
        """return substring, eclusive prefix or suffix"""
        str_p = str.rfind(prefix) + len(prefix)
        end_p = str.rfind(suffix)
        return str[str_p:end_p]

    def reorganize_extracted_data(self,
                                  tmp_data_path,
                                  remove_input_data_cache=False):
        root_path = os.path.dirname(os.path.normpath(tmp_data_path))
        output_path = None

        config_yaml = ConfigYaml()
        task_name = self.config.io_config.task_name
        if task_name == 'lidar_to_gnss':
            subfolders = [
                x for x in get_subfolder_list(tmp_data_path)
                if '_apollo_sensor_' in x or '_localization_pose' in x
            ]
            odometry_subfolders = [
                x for x in subfolders if '_odometry' in x or '_pose' in x
            ]
            lidar_subfolders = [x for x in subfolders if '_PointCloud2' in x]
            print(lidar_subfolders)
            print(odometry_subfolders)
            if len(lidar_subfolders) == 0 or len(odometry_subfolders) != 1:
                raise ValueError(('one odometry and more than 0 lidar(s)'
                                  'sensor are needed for sensor calibration'))
            odometry_subfolder = odometry_subfolders[0]
            yaml_list = []
            gnss_name = 'novatel'
            multi_lidar_out_path = os.path.join(
                root_path, 'multi_lidar_to_gnss_calibration')
            output_path = multi_lidar_out_path

            for lidar in lidar_subfolders:
                # get the lidar name from folder name string
                lidar_name = Extractor.get_substring(str=lidar,
                                                     prefix='_apollo_sensor_',
                                                     suffix='_PointCloud2')

                # reorganize folder structure: each lidar has its raw data,
                # corresponding odometry and configuration yaml file

                if not Extractor.process_dir(multi_lidar_out_path, 'create'):
                    raise ValueError(
                        f'Failed to create directory: {multi_lidar_out_path}')
                lidar_in_path = os.path.join(tmp_data_path, lidar)
                lidar_out_path = os.path.join(multi_lidar_out_path, lidar)
                if not os.path.exists(lidar_out_path):
                    shutil.copytree(lidar_in_path, lidar_out_path)
                odometry_in_path = os.path.join(tmp_data_path,
                                                odometry_subfolder)
                odometry_out_path = os.path.join(multi_lidar_out_path,
                                                 odometry_subfolder)
                if not os.path.exists(odometry_out_path):
                    shutil.copytree(odometry_in_path, odometry_out_path)
                generated_config_yaml = os.path.join(
                    tmp_data_path, lidar_name + '_' + 'sample_config.yaml')
                config_yaml.generate_task_config_yaml(
                    task_name=task_name,
                    source_sensor=lidar_name,
                    dest_sensor=gnss_name,
                    source_folder=lidar,
                    dest_folder=odometry_subfolder,
                    out_config_file=generated_config_yaml)
                print(f'lidar {lidar_name} calibration data and configuration'
                      ' are generated.')
                yaml_list.append(generated_config_yaml)

            out_data = {
                'calibration_task': task_name,
                'destination_sensor': gnss_name,
                'odometry_file': odometry_subfolder + '/odometry'
            }
            sensor_files_directory_list = []
            source_sensor_list = []
            transform_list = []
            for i in range(len(yaml_list)):
                with open(yaml_list[i], 'r') as f:
                    data = yaml.safe_load(f)
                    sensor_files_directory_list.append(
                        data['sensor_files_directory'])
                    source_sensor_list.append(data['source_sensor'])
                    transform_list.append(data['transform'])
            out_data['sensor_files_directory'] = sensor_files_directory_list
            out_data['source_sensor'] = source_sensor_list
            out_data['transform'] = transform_list
            out_data['main_sensor'] = source_sensor_list[0]

            table = preprocess_table_pb2.PreprocessTable()
            user_config = os.path.join(FLAGS.root_dir, 'config',
                                       'lidar_to_gnss_user.config')
            if os.path.exists(user_config):
                try:
                    get_pb_from_text_file(user_config, table)
                except text_format.ParseError:
                    print(f'Error: Cannot parse {user_config} as text proto')

                if table.HasField("main_sensor"):
                    out_data['main_sensor'] = table.main_sensor

            multi_lidar_yaml = os.path.join(multi_lidar_out_path,
                                            'sample_config.yaml')
            with open(multi_lidar_yaml, 'w') as f:
                yaml.safe_dump(out_data, f)

        elif task_name == 'camera_to_lidar':
            # data selection.
            pair_data_folder_name = 'camera-lidar-pairs'
            cameras, lidar = select_static_image_pcd(
                path=tmp_data_path,
                min_distance=5,
                stop_times=4,
                wait_time=3,
                check_range=50,
                image_static_diff_threshold=0.005,
                output_folder_name=pair_data_folder_name,
                image_suffix='.jpg',
                pcd_suffix='.pcd')
            lidar_name = Extractor.get_substring(str=lidar,
                                                 prefix='_apollo_sensor_',
                                                 suffix='_PointCloud2')
            for camera in cameras:
                camera_name = Extractor.get_substring(str=camera,
                                                      prefix='_apollo_sensor_',
                                                      suffix='_image')
                out_path = os.path.join(
                    root_path,
                    camera_name + '_to_' + lidar_name + '_calibration')
                output_path = out_path
                if not Extractor.process_dir(out_path, 'create'):
                    raise ValueError(f'Failed to create directory: {out_path}')
                # reorganize folder structure: each camera has its images,
                # corresponding lidar pointclouds, camera initial extrinsics,
                # intrinsics, and configuration yaml file

                in_pair_data_path = os.path.join(tmp_data_path, camera,
                                                 pair_data_folder_name)
                out_pair_data_path = os.path.join(out_path,
                                                  pair_data_folder_name)
                shutil.copytree(in_pair_data_path, out_pair_data_path)
                generated_config_yaml = os.path.join(out_path,
                                                     'sample_config.yaml')
                config_yaml.generate_task_config_yaml(
                    task_name=task_name,
                    source_sensor=camera_name,
                    dest_sensor=lidar_name,
                    source_folder=None,
                    dest_folder=None,
                    out_config_file=generated_config_yaml)
        elif task_name == 'radar_to_gnss':
            print('not ready. stay tuned')
        else:
            raise ValueError(
                f'Unsupported data extraction task for {task_name}')

        if remove_input_data_cache:
            print(f'removing the cache at {tmp_data_path}')
            os.system(f'rm -rf {tmp_data_path}')
        return output_path

    def sanity_check_path(self, path):
        """Sanity check wrapper"""
        result, log_str = sanity_check(path)
        if result is True:
            self.progress.percentage = 100.0
            self.progress.status = preprocess_table_pb2.Status.SUCCESS
        else:
            self.progress.status = preprocess_table_pb2.Status.FAIL
        self.progress.log_string = log_str
        self.writer.write(self.progress)
        time.sleep(0.5)

    def create_tmp_directory(self):
        """Create directory to save the extracted data use time now() as folder name"""
        output_relative_path = self.config.io_config.task_name + datetime.now(
        ).strftime("-%Y-%m-%d-%H-%M") + '/tmp/'

        output_abs_path = os.path.join(self.config.io_config.output_path,
                                       output_relative_path)
        ret = self.process_dir(output_abs_path, 'create')
        if not ret:
            raise ValueError(
                f'Failed to create extrated data directory: {output_abs_path}')
        return output_abs_path


def main(argv):
    """Main function"""

    cyber.init("data_extractor")
    extractor = Extractor()

    valid_record_list = extractor.validate_record_files(kword='.record.')

    channels, extraction_rates = extractor.parse_channel_config()
    print(f'parsing the following channels: {channels}')

    output_tmp_path = extractor.create_tmp_directory()
    extractor.extract_data(valid_record_list, output_tmp_path, channels,
                           extraction_rates)

    output_abs_path = extractor.reorganize_extracted_data(
        tmp_data_path=output_tmp_path, remove_input_data_cache=True)

    print('Data extraction is completed successfully!')
    extractor.sanity_check_path(output_abs_path)
    cyber.shutdown()
    sys.exit(0)


if __name__ == '__main__':
    # root_path = '/apollo/data/extracted_data/MKZ5-2019-05-15/lidar_to_gnss-2019-11-25-11-02/tmp'
    # task_name = 'lidar_to_gnss'
    # root_path = '/apollo/data/extracted_data/udevl002-2019-06-14/camera_to_lidar-2019-11-26-19-49/tmp'
    # task_name = 'camera_to_lidar'
    # reorganize_extracted_data(tmp_data_path=root_path, task_name=task_name)
    app.run(main)
