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
Extract messages of specified topics from data record file,
and populate them into frames with JSON format

Usage:
    populate_frames.py --input_file=a.record

See the gflags for more optional args.
"""
import os
import sys
import time

import cv2
import gflags
import glog
import numpy as np
import yaml
from google.protobuf.json_format import MessageToJson

apollo_root = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '../../../..'))
py_proto_path = os.path.join(apollo_root, 'py_proto')
sys.path.append(py_proto_path)

from cybertron import cybertron
from cybertron import record
from modules.data.proto import frame_pb2
from modules.drivers.proto.sensor_image_pb2 import Image
from modules.drivers.proto.sensor_image_pb2 import CompressedImage
from modules.drivers.proto.pointcloud_pb2 import PointCloud
from modules.drivers.proto.conti_radar_pb2 import ContiRadar
from modules.localization.proto.localization_pb2 import LocalizationEstimate

# Requried flags.
gflags.DEFINE_string('input_file', None, 'Input record file path.')

# Optional flags.
gflags.DEFINE_string('output_path', './', 'Output folder path.')
gflags.DEFINE_integer('maximum_frame_count', 100, 'Maximum frame count.')
gflags.DEFINE_integer('frame_size', 10, 'The size of frame in nanoseconds.')

# Stable flags which rarely change.
gflags.DEFINE_string('pointcloud_128_channel', 
                     '/apollo/sensor/lidar128/PointCloud2',
                     'point cloud 128-line channel.')
gflags.DEFINE_string('pointcloud_128_extrinsics',
                     'apollo/modules/calibration/data/'\
                     'mkz_example/velodyne_params/'\
                     'velodyne64_novatel_extrinsics_example.yaml',
                     'YAML settings for 128 line point cloud')
gflags.DEFINE_string('pointcloud_front_channel', 
                     '/apollo/sensor/lidar16/front/center/PointCloud2',
                     'point cloud front channel.')
gflags.DEFINE_string('pointcloud_front_extrinsics',
                     'apollo/modules/calibration/data/'\
                     'mkz_example/velodyne_params/'\
                     'velodyne64_novatel_extrinsics_example.yaml',
                     'YAML settings for front point cloud')
gflags.DEFINE_string('pointcloud_rear_left_channel', 
                     '/apollo/sensor/lidar16/rear/left/PointCloud2',
                     'point cloud rear left channel.')
gflags.DEFINE_string('pointcloud_rear_left_extrinsics',
                     'apollo/modules/calibration/data/'\
                     'mkz_example/velodyne_params/'\
                     'velodyne64_novatel_extrinsics_example.yaml',
                     'YAML settings for rear left point cloud')
gflags.DEFINE_string('pointcloud_rear_right_channel', 
                     '/apollo/sensor/lidar16/rear/right/PointCloud2',
                     'point cloud rear right channel.')       
gflags.DEFINE_string('pointcloud_rear_right_extrinsics',
                     'apollo/modules/calibration/data/'\
                     'mkz_example/velodyne_params/'\
                     'velodyne64_novatel_extrinsics_example.yaml',
                     'YAML settings for rear right point cloud')                                  
gflags.DEFINE_string('image_front_6mm_channel', 
                     '/apollo/sensor/camera/front_6mm/image/compressed',
                     'image front 6mm channel.')
gflags.DEFINE_string('image_front_6mm_intrinsics',
                     'apollo/modules/calibration/data/'\
                     'mkz_example/camera_params/'\
                     'front_camera_intrinsics.yaml',
                     'intrinsics settings for front 6mm camera')
gflags.DEFINE_string('image_front_6mm_extrinsics',
                     'apollo/modules/calibration/data/'\
                     'mkz_example/camera_params/'\
                     'front_camera_extrinsics.yaml',
                     'extrinsics settings for front 6mm camera')
gflags.DEFINE_string('image_front_12mm_channel', 
                     '/apollo/sensor/camera/front_12mm/image/compressed',
                     'image front 12mm channel.')
gflags.DEFINE_string('image_front_12mm_intrinsics',
                     'apollo/modules/calibration/data/'\
                     'mkz_example/camera_params/'\
                     'long_camera_intrinsics.yaml',
                     'intrinsics settings for front 12mm camera')
gflags.DEFINE_string('image_front_12mm_extrinsics',
                     'apollo/modules/calibration/data/'\
                     'mkz_example/camera_params/'\
                     'long_camera_extrinsics.yaml',
                     'extrinsics settings for front 12mm camera')
gflags.DEFINE_string('image_left_fisheye_channel', 
                     '/apollo/sensor/camera/left_fisheye/image/compressed',
                     'image left fisheye channel.')
gflags.DEFINE_string('image_left_fisheye_intrinsics',
                     'apollo/modules/calibration/data/'\
                     'mkz_example/camera_params/'\
                     'short_camera_intrinsics.yaml',
                     'intrinsics settings for left fisheye camera')
gflags.DEFINE_string('image_left_fisheye_extrinsics',
                     'apollo/modules/calibration/data/'\
                     'mkz_example/camera_params/'\
                     'short_camera_extrinsics.yaml',
                     'extrinsics settings for left fisheye camera')
gflags.DEFINE_string('image_right_fisheye_channel', 
                     '/apollo/sensor/camera/right_fisheye/image/compressed',
                     'image right fisheye channel.')
gflags.DEFINE_string('image_right_fisheye_intrinsics',
                     'apollo/modules/calibration/data/'\
                     'mkz_example/camera_params/'\
                     'short_camera_intrinsics.yaml',
                     'intrinsics settings for right fisheye camera')
gflags.DEFINE_string('image_right_fisheye_extrinsics',
                     'apollo/modules/calibration/data/'\
                     'mkz_example/camera_params/'\
                     'short_camera_extrinsics.yaml',
                     'extrinsics settings for right fisheye camera')
gflags.DEFINE_string('image_rear_6mm_channel', 
                     '/apollo/sensor/camera/rear_6mm/image/compressed',
                     'image rear channel.')
gflags.DEFINE_string('image_rear_6mm_intrinsics',
                     'apollo/modules/calibration/data/'\
                     'mkz_example/camera_params/'\
                     'short_camera_intrinsics.yaml',
                     'intrinsics settings for rear 6mm camera')
gflags.DEFINE_string('image_rear_6mm_extrinsics',
                     'apollo/modules/calibration/data/'\
                     'mkz_example/camera_params/'\
                     'short_camera_extrinsics.yaml',
                     'extrinsics settings for rear 6mm camera')                    
gflags.DEFINE_string('pose_channel', 
                     '/apollo/localization/pose',
                     'pose channel.')
gflags.DEFINE_string('radar_front_channel', 
                     '/apollo/sensor/radar/front',
                     'radar front channel.')
gflags.DEFINE_string('radar_front_extrinsics',
                     'apollo/modules/calibration/data/'\
                     'mkz_example/radar_params/'\
                     'radar_front_extrinsics.yaml',
                     'YAML settings for front radar')
gflags.DEFINE_string('radar_rear_channel', 
                     '/apollo/sensor/radar/rear',
                     'radar rear channel.')
gflags.DEFINE_string('radar_rear_extrinsics',
                     'apollo/modules/calibration/data/'\
                     'mkz_example/radar_params/'\
                     'radar_front_extrinsics.yaml',
                     'YAML settings for rear radar')
gflags.DEFINE_string('image_url_path', 
                     'https://s3-us-west-1.amazonaws.com/scale-frames/images',
                     'public url for storing images.')

# Map channels to processing functions
g_channel_process_map = {}

def load_yaml_settings(yaml_file_name):
    """Load settings from YAML config file."""
    if yaml_file_name is None:
        return None
    yaml_file_name = os.path.join(apollo_root, yaml_file_name)
    yaml_file = open(yaml_file_name)
    return yaml.safe_load(yaml_file)

def dump_yuyv_img(data, output_dir, seq):
    """Dump image bytes with YUYV format to photo."""
    dtype = 'uint8'
    dtype = np.dtype(dtype)
    img = np.ndarray(shape=(1080,1920,2), dtype=dtype, buffer=data)
    img = cv2.cvtColor(img, cv2.COLOR_YUV2BGR_YUYV)
    cv2.imwrite('{}/image-{}.jpg'.format(output_dir, seq), img)

def dump_rgb_img(data, output_dir, seq):
    """Dump image bytes with RGB format to photo."""
    dtype = 'uint8'
    dtype = np.dtype(dtype)
    img = np.ndarray(shape=(1080,1920,3), dtype=dtype, buffer=data)
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    cv2.imwrite('{}/image-{}.jpg'.format(output_dir, seq), img)

def dump_img_bin(data, output_dir, seq):
    """Dump image bytes to binary file."""
    with open('{}/image_bin-{}'.format(output_dir, seq), 'wb') as bin_file:
        bin_file.write(data)

def dump_compressed_img(data, output_dir, seq):
    """Dump compressed image bytes with RGB format to photo."""
    img = np.asarray(bytearray(data), dtype="uint8")
    img = cv2.imdecode(img, cv2.IMREAD_COLOR)
    cv2.imwrite('{}/image-{}.jpg'.format(output_dir, seq), img)

def point3d_to_matrix(x, y, z):
    """Convert a 3-items array to one row matrix."""
    point_mat = np.zeros(shape=(1,3), dtype=float)
    point_mat[0] = [x,y,z]
    return point_mat

def quaternion_to_roation(q0, q1, q2, q3):
    """Convert quaternion vector to 3x3 rotation matrix."""
    rotation_mat = np.zeros(shape=(3,3), dtype=float)
    rotation_mat[0][0] = q0**2 + q1**2 - q2**2 - q3**2
    rotation_mat[0][1] = 2 * (q1*q2 - q0*q3)
    rotation_mat[0][2] = 2 * (q1*q3 + q0*q2)
    rotation_mat[1][0] = 2 * (q1*q2 + q0*q3)
    rotation_mat[1][1] = q0**2 - q1**2 + q2**2 - q3**2
    rotation_mat[1][2] = 2 * (q2*q3 - q0*q1)
    rotation_mat[2][0] = 2 * (q1*q3 - q0*q2)
    rotation_mat[2][1] = 2 * (q2*q3 + q0*q1)
    rotation_mat[2][2] = q0**2 - q1**2 - q2**2 + q3**2
    return rotation_mat

def transform_coordinate(x, y, z, R, T):
    """Transform coordinate system according to rotation and translation."""
    point_mat = point3d_to_matrix(x, y, z)
    #point_rotation = np.matmul(point_mat, R)
    point_rotation = np.dot(point_mat, R)
    point_translation = point_rotation + T
    return point_translation

class Sensor(object):
    """Sensor class representing various of sensors."""
    def __init__(self, channel, intrinsics, extrinsics):
        """Constructor."""
        self._channel = channel
        self._intrinsics = load_yaml_settings(intrinsics)
        self._extrinsics = load_yaml_settings(extrinsics)
        self._current = None
        self._current_list = []
        g_channel_process_map[self._channel] = self
 
    def process(self, message):
        """Processing function."""
        pass
    
    def construct_frame(self, frame):
        """Construct the current frame with sensor's data."""
        if len(self._current_list) != 0:
            self._current = self._current_list[-1]
        elif self._current is not None:
            self._current_list.append(self._current)
        self.offer_data(frame)
        self._current_list = []
    
    def offer_data(self, frame):
        """Processing function."""
        pass

class PointCloudSensor(Sensor):
    """Lidar sensor that hold pointcloud data."""
    def __init__(self, channel, intrinsics, extrinsics, pose):
        """Initialization."""
        super(PointCloudSensor, self).__init__(channel, intrinsics, extrinsics)
        self._first_x = None 
        self._first_y = None 
        self._First_z = None 
        self._pose = pose

    def process(self, message):
        """Process PointCloud message."""
        point_cloud = PointCloud()
        point_cloud.ParseFromString(message)
        for point in point_cloud.point:
            if self._pose._position is None or self._pose._orientation is None:
                break
            trans_point = self.transform_pointcloud((point.x, point.y, point.z))
            #trans_point = (point.x, point.y, point.z)
            if self._first_x is None:
                self._first_x = trans_point[0]
                self._first_y = trans_point[1]
                self._first_z = trans_point[2]
            vector4 = frame_pb2.Vector4()
            vector4.x = trans_point[0] - self._first_x
            vector4.y = trans_point[1] - self._first_y
            vector4.z = trans_point[2] - self._first_z
            vector4.i = point.intensity
            self._current_list.append(vector4)
        return point_cloud.header.timestamp_sec * 1e9

    def offer_data(self, frame):
        """Provide data to current frame."""
        for vector4 in self._current_list:
            point = frame.points.add()
            point.CopyFrom(vector4)
        frame.device_position.x = \
            self._extrinsics['transform']['translation']['x']
        frame.device_position.y = \
            self._extrinsics['transform']['translation']['y']
        frame.device_position.z = \
            self._extrinsics['transform']['translation']['z']
        frame.device_heading.x = \
            self._extrinsics['transform']['rotation']['x']
        frame.device_heading.y = \
            self._extrinsics['transform']['rotation']['y']
        frame.device_heading.z = \
            self._extrinsics['transform']['rotation']['z']
        frame.device_heading.w = \
            self._extrinsics['transform']['rotation']['w']

    def transform_pointcloud(self, point):
        """Tranform from Lidar local to world coordinates."""
        rotation_lidar_to_imu = self._extrinsics['transform']['rotation']
        rotation_imu_to_world = self._pose._orientation
        translation_lidar_to_imu = self._extrinsics['transform']['translation']
        translation_imu_to_world = self._pose._position
        trans_point = transform_coordinate(
                point[0],
                point[1], 
                point[2], 
                quaternion_to_roation(
                    rotation_lidar_to_imu['w'],
                    rotation_lidar_to_imu['x'],
                    rotation_lidar_to_imu['y'],
                    rotation_lidar_to_imu['z']),
                point3d_to_matrix(
                    translation_lidar_to_imu['x'],
                    translation_lidar_to_imu['y'],
                    translation_lidar_to_imu['z']))
        trans_point = transform_coordinate(
            trans_point[0][0],
            trans_point[0][1],
            trans_point[0][2],
            quaternion_to_roation(
                rotation_imu_to_world.qw,
                rotation_imu_to_world.qx,
                rotation_imu_to_world.qy,
                rotation_imu_to_world.qz),
	    point3d_to_matrix(
                translation_imu_to_world.x,
                translation_imu_to_world.y,
                translation_imu_to_world.z))
        return (trans_point[0][0],trans_point[0][1],trans_point[0][2])

class RadarSensor(Sensor):
    """Radar sensor that hold radar data."""
    def __init__(self, channel, intrinsics, extrinsics):
        """Initialization."""
        super(RadarSensor, self).__init__(channel, intrinsics, extrinsics)

    def process(self, message):
        """Processing radar message."""
        radar = ContiRadar()
        radar.ParseFromString(message)
        for point in radar.contiobs:
            radar_point = frame_pb2.RadarPoint()
            radar_point.position.x = point.longitude_dist
            radar_point.position.y = point.lateral_dist
            radar_point.position.z = 0
            radar_point.direction.x = point.longitude_vel
            radar_point.direction.y = point.lateral_vel
            radar_point.direction.z = 0
            self._current_list.append(radar_point)
        return radar.header.timestamp_sec * 1e9
    
    def offer_data(self, frame):
        """Provide data to current frame."""
        for radar in self._current_list:
            new_radar = frame.radar_points.add()
            new_radar.CopyFrom(radar)

class ImageSensor(Sensor):
    """Image sensor that hold radar data."""
    _image_seq = 0

    def __init__(self, channel, intrinsics, extrinsics, path, url):
        """Initialization."""
        super(ImageSensor, self).__init__(channel, intrinsics, extrinsics)
        self._output_dir = path
        self._image_url = url

    def process(self, message):
        """Processing image message."""
        image = CompressedImage()
        image.ParseFromString(message)
        camera_image = frame_pb2.CameraImage()
        camera_image.timestamp = image.header.timestamp_sec
        dump_img_bin(image.data, self._output_dir, ImageSensor._image_seq)
        camera_image.image_url = '{}/pic-{}.jpg'.format(
            self._image_url, 
            ImageSensor._image_seq)
        ImageSensor._image_seq = ImageSensor._image_seq + 1
        camera_image.k1 = self._intrinsics['D'][0]
        camera_image.k2 = self._intrinsics['D'][1]
        camera_image.k3 = self._intrinsics['D'][4]
        camera_image.p1 = self._intrinsics['D'][2]
        camera_image.p2 = self._intrinsics['D'][3]
        camera_image.skew = self._intrinsics['K'][1]
        camera_image.fx = self._intrinsics['K'][0]
        camera_image.fy = self._intrinsics['K'][4]
        camera_image.cx = self._intrinsics['K'][2]
        camera_image.cy = self._intrinsics['K'][5]
        camera_image.position.x = \
            self._extrinsics['transform']['translation']['x']
        camera_image.position.y = \
            self._extrinsics['transform']['translation']['y']
        camera_image.position.z = \
            self._extrinsics['transform']['translation']['z']
        camera_image.heading.x = self._extrinsics['transform']['rotation']['x']
        camera_image.heading.y = self._extrinsics['transform']['rotation']['y']
        camera_image.heading.z = self._extrinsics['transform']['rotation']['z']
        camera_image.heading.w = self._extrinsics['transform']['rotation']['w']
        self._current_list.append(camera_image)
        return image.header.timestamp_sec * 1e9
    
    def offer_data(self, frame):
        """Provide data to current frame."""
        for image in self._current_list:
            new_image = frame.images.add()
            new_image.CopyFrom(image)

class GpsSensor(Sensor):
    """GPS sensor that hold pose data."""
    def __init__(self, channel, intrinsics, extrinsics):
        """Initialization."""
        super(GpsSensor, self).__init__(channel, intrinsics, extrinsics)
        self._current = frame_pb2.GPSPose()
        self._position = None
        self._orientation = None
        
    def process(self, message):
        """Process Pose message."""
        localization = LocalizationEstimate()
        localization.ParseFromString(message)
        self._current.lat = localization.pose.position.x
        self._current.lon = localization.pose.position.y
        self._current.bearing = localization.pose.orientation.qw
        self._position = localization.pose.position
        self._orientation = localization.pose.orientation
        return localization.header.timestamp_sec * 1e9

    def offer_data(self, frame):
        """Provide data to current frame."""
        frame.device_gps_pose.CopyFrom(self._current)
        
class FramePopulator:
    """Extract sensors data from record file, and populate to JSON."""
    def __init__(self, args):
        self._args = args
        self._current_frame_id = 0 
        self._current_frame = None
        self._frame_count = 0
        self.set_sensors(args)
    
    def set_sensors(self, args):
        """Set all sensor instances."""
        self._gps_pose = GpsSensor(args.pose_channel, None, None)
        self._pointcloud = PointCloudSensor(
            args.pointcloud_128_channel,
            None,
            args.pointcloud_128_extrinsics,
            self._gps_pose
        )
        self._image_front_6mm = ImageSensor(
            args.image_front_6mm_channel,
            args.image_front_6mm_intrinsics,
            args.image_front_6mm_extrinsics,
            args.output_path,
            args.image_url_path)
        self._image_front_12mm = ImageSensor(
            args.image_front_12mm_channel,
            args.image_front_12mm_intrinsics,
            args.image_front_12mm_extrinsics,
            args.output_path,
            args.image_url_path)
        self._image_left_fisheye = ImageSensor(
            args.image_left_fisheye_channel,
            args.image_left_fisheye_intrinsics,
            args.image_left_fisheye_extrinsics,
            args.output_path,
            args.image_url_path)
        self._image_right_fisheye = ImageSensor(
            args.image_right_fisheye_channel,
            args.image_right_fisheye_intrinsics,
            args.image_right_fisheye_extrinsics,
            args.output_path,
            args.image_url_path)
        self._image_rear = ImageSensor(
            args.image_rear_6mm_channel,
            args.image_rear_6mm_intrinsics,
            args.image_rear_6mm_extrinsics,
            args.output_path,
            args.image_url_path)
        self._radar_front = RadarSensor(
            args.radar_front_channel,
            None,
            args.radar_front_extrinsics
        )
        self._radar_rear = RadarSensor(
            args.radar_rear_channel,
            None,
            args.radar_rear_extrinsics
        )

    def construct_final_frame(self):
        """Construct the current frame to make it ready for dumping."""
        for sensor in g_channel_process_map.values():
            sensor.construct_frame(self._current_frame)
        self._current_frame.timestamp = self._current_frame_id / 1e9

    def dump_to_json_file(self):
        """Dump the frame content to JSON file."""
        # Construct the final frame before dumping
        self.construct_final_frame()
        self._frame_count = self._frame_count + 1
        file_name = os.path.join(self._args.output_path, 
            'frame-{}.json'.format(self._frame_count))
        jsonObj = MessageToJson(self._current_frame, False, True)
        with open(file_name, 'w') as outfile:
           outfile.write(jsonObj)

    def check_frame(self, frame_id):
        """Check if a new frame needs to be created."""
        if self._current_frame_id == 0:
            self._current_frame_id = frame_id
        elif frame_id - self._current_frame_id >= self._args.frame_size:
            glog.info('#current frame {}, changing to {}'.format(
                self._current_frame_id, frame_id))
            if self._current_frame != None:
                glog.info('#dumping frame %d' %(self._current_frame_id))
                self.dump_to_json_file()
            self._current_frame = frame_pb2.Frame()
            self._current_frame_id = frame_id

    def process_record_file(self):
        """Read record file and extract the message with specified channels"""
        freader = record.RecordReader(self._args.input_file)
        time.sleep(1)
        glog.info('#processing record file {}'.format(self._args.input_file))
        for channel, message, _type, _timestamp in freader.read_messages():
            if self._frame_count >= self._args.maximum_frame_count:
                glog.info('#reached the maximum frame count, exiting now')
                return
            if channel in g_channel_process_map:
                frame_id = g_channel_process_map[channel].process(message)
                self.check_frame(frame_id)
        # Dump the last frame if has not reached maximum
        if self._frame_count < self._args.maximum_frame_count:
            self.dump_to_json_file()

def main():
    """Entry point."""
    gflags.FLAGS(sys.argv)
    frame_populator = FramePopulator(gflags.FLAGS)
    frame_populator.process_record_file()
    return

if __name__ == '__main__':
    main()

