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
import math
import yaml
from copy import deepcopy
from google.protobuf.json_format import MessageToJson

apollo_root = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '../../../..'))
py_proto_path = os.path.join(apollo_root, 'py_proto')
sys.path.append(py_proto_path)

from cybertron import cybertron
from cybertron import record

from modules.data.proto import frame_pb2
from modules.common.proto.geometry_pb2 import Point3D
from modules.common.proto.geometry_pb2 import Quaternion
from modules.drivers.proto.conti_radar_pb2 import ContiRadar
from modules.drivers.proto.pointcloud_pb2 import PointCloud
from modules.drivers.proto.sensor_image_pb2 import CompressedImage
from modules.drivers.proto.sensor_image_pb2 import Image
from modules.localization.proto.localization_pb2 import LocalizationEstimate

# Requried flags.
gflags.DEFINE_string('input_file', None, 'Input record file path.')

# Optional flags.
gflags.DEFINE_string('output_path', './', 'Output folder path.')
gflags.DEFINE_integer('maximum_frame_count', 2, 'Maximum frame count.')
gflags.DEFINE_integer('skip_frame_count', 10, 'First number of frames to skip.')
gflags.DEFINE_integer('skip_frame_rate', 1, 'Skip every number of frames.')
gflags.DEFINE_integer('initial_frame_seq', 0, 'Initial sequence of frames')
gflags.DEFINE_integer('initial_image_seq', 0, 'Initial sequence of images')
gflags.DEFINE_boolean('reset_stationary_pole', True, 'Whether reset original')

# Stable flags which rarely change.
gflags.DEFINE_string('pointcloud_128_channel', 
                     '/apollo/sensor/lidar128/compensator/PointCloud2',
                     'point cloud 128-line channel.')
gflags.DEFINE_string('pointcloud_128_extrinsics',
                     'apollo/modules/calibration/data/'\
                     'mkz_example/lidar_internal/'\
                     'vls128_novatel_extrinsics.yaml',
                     'YAML settings for 128 line point cloud')
gflags.DEFINE_string('pointcloud_front_channel', 
                     '/apollo/sensor/lidar16/front/center/PointCloud2',
                     'point cloud front channel.')
gflags.DEFINE_string('pointcloud_front_extrinsics',
                     'apollo/modules/calibration/data/'\
                     'mkz_example/lidar_internal/'\
                     'vlp16_front_center_novatel_extrinsics.yaml',
                     'YAML settings for front point cloud')
gflags.DEFINE_string('pointcloud_rear_left_channel', 
                     '/apollo/sensor/lidar16/rear/left/PointCloud2',
                     'point cloud rear left channel.')
gflags.DEFINE_string('pointcloud_rear_left_extrinsics',
                     'apollo/modules/calibration/data/'\
                     'mkz_example/lidar_internal/'\
                     'vlp16_rear_left_novatel_extrinsics.yaml',
                     'YAML settings for rear left point cloud')
gflags.DEFINE_string('pointcloud_rear_right_channel', 
                     '/apollo/sensor/lidar16/rear/right/PointCloud2',
                     'point cloud rear right channel.')       
gflags.DEFINE_string('pointcloud_rear_right_extrinsics',
                     'apollo/modules/calibration/data/'\
                     'mkz_example/lidar_internal/'\
                     'vlp16_rear_right_novatel_extrinsics.yaml',
                     'YAML settings for rear right point cloud')                                  
gflags.DEFINE_string('image_front_6mm_channel', 
                     '/apollo/sensor/camera/front_6mm/image/compressed',
                     'image front 6mm channel.')
gflags.DEFINE_string('image_front_6mm_intrinsics',
                     'apollo/modules/calibration/data/'\
                     'mkz_example/camera_internal/'\
                     'front_center_6mm_intrinsics.yaml',
                     'intrinsics settings for front 6mm camera')
gflags.DEFINE_string('image_front_6mm_extrinsics',
                     'apollo/modules/calibration/data/'\
                     'mkz_example/camera_internal/'\
                     'front_center_6mm_velodyne128_extrinsics.yaml',
                     'extrinsics settings for front 6mm camera')
gflags.DEFINE_string('image_front_12mm_channel', 
                     '/apollo/sensor/camera/front_12mm/image/compressed',
                     'image front 12mm channel.')
gflags.DEFINE_string('image_front_12mm_intrinsics',
                     'apollo/modules/calibration/data/'\
                     'mkz_example/camera_internal/'\
                     'front_center_12mm_intrinsics.yaml',
                     'intrinsics settings for front 12mm camera')
gflags.DEFINE_string('image_front_12mm_extrinsics',
                     'apollo/modules/calibration/data/'\
                     'mkz_example/camera_internal/'\
                     'front_center_12mm_velodyne128_extrinsics.yaml',
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
                     'mkz_example/camera_internal/'\
                     'fisheye_left_extrinsics.yaml',
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
                     'mkz_example/camera_internal/'\
                     'fisheye_right_extrinsics.yaml',
                     'extrinsics settings for right fisheye camera')
gflags.DEFINE_string('image_rear_6mm_channel', 
                     '/apollo/sensor/camera/rear_6mm/image/compressed',
                     'image rear channel.')
gflags.DEFINE_string('image_rear_6mm_intrinsics',
                     'apollo/modules/calibration/data/'\
                     'mkz_example/camera_internal/'\
                     'rear_center_6mm_intrinsics.yaml',
                     'intrinsics settings for rear 6mm camera')
gflags.DEFINE_string('image_rear_6mm_extrinsics',
                     'apollo/modules/calibration/data/'\
                     'mkz_example/camera_internal/'\
                     'rear_center_6mm_velodyne128_extrinsics.yaml',
                     'extrinsics settings for rear 6mm camera')                    
gflags.DEFINE_string('pose_channel', 
                     '/apollo/localization/pose',
                     'pose channel.')
gflags.DEFINE_string('radar_front_channel', 
                     '/apollo/sensor/radar/front',
                     'radar front channel.')
gflags.DEFINE_string('radar_front_extrinsics',
                     'apollo/modules/calibration/data/'\
                     'mkz_example/radar_internal/'\
                     'radar_front_extrinsics.yaml',
                     'YAML settings for front radar')
gflags.DEFINE_string('radar_rear_channel', 
                     '/apollo/sensor/radar/rear',
                     'radar rear channel.')
gflags.DEFINE_string('radar_rear_extrinsics',
                     'apollo/modules/calibration/data/'\
                     'mkz_example/radar_internal/'\
                     'radar_rear_extrinsics.yaml',
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

def point3d_to_matrix(P):
    """Convert a 3-items array to 4*1 matrix."""
    mat = np.zeros(shape=(4,1), dtype=float) 
    mat = np.array([[P.x],[P.y],[P.z],[1]])
    return mat

def quaternion_to_roation(Q):
    """Convert quaternion vector to 3x3 rotation matrix."""
    rotation_mat = np.zeros(shape=(3,3), dtype=float)
    rotation_mat[0][0] = Q.qw**2 + Q.qx**2 - Q.qy**2 - Q.qz**2
    rotation_mat[0][1] = 2 * (Q.qx*Q.qy - Q.qw*Q.qz)
    rotation_mat[0][2] = 2 * (Q.qx*Q.qz + Q.qw*Q.qy)
    rotation_mat[1][0] = 2 * (Q.qx*Q.qy + Q.qw*Q.qz)
    rotation_mat[1][1] = Q.qw**2 - Q.qx**2 + Q.qy**2 - Q.qz**2
    rotation_mat[1][2] = 2 * (Q.qy*Q.qz - Q.qw*Q.qx)
    rotation_mat[2][0] = 2 * (Q.qx*Q.qz - Q.qw*Q.qy)    
    rotation_mat[2][1] = 2 * (Q.qy*Q.qz + Q.qw*Q.qx)    
    rotation_mat[2][2] = Q.qw**2 - Q.qx**2 - Q.qy**2 + Q.qz**2
    return rotation_mat

def rotation_to_quaternion(R):
    """Convert 3x3 rotation matrix to quaternion vector."""
    q = Quaternion()
    q.qx = np.absolute(np.sqrt(1+R[0][0]-R[1][1]-R[2][2])) * \
        np.sign(R[2][1]-R[1][2]) * 0.5
    q.qy = np.absolute(np.sqrt(1-R[0][0]+R[1][1]-R[2][2])) * \
        np.sign(R[0][2]-R[2][0]) * 0.5 
    q.qz = np.absolute(np.sqrt(1-R[0][0]-R[1][1]+R[2][2])) * \
        np.sign(R[1][0]-R[0][1]) * 0.5
    q.qw = np.sqrt(1 - q.qx * q.qx - q.qy * q.qy - q.qz * q.qz)
    return q

def generate_transform(Q, D):
    """Generate a matrix with rotation and deviation/translation."""
    tranform = np.zeros(shape=(4,4), dtype=float)
    tranform[0][0] = Q.qw**2 + Q.qx**2 - Q.qy**2 - Q.qz**2
    tranform[0][1] = 2 * (Q.qx*Q.qy - Q.qw*Q.qz)
    tranform[0][2] = 2 * (Q.qx*Q.qz + Q.qw*Q.qy)
    tranform[1][0] = 2 * (Q.qx*Q.qy + Q.qw*Q.qz)
    tranform[1][1] = Q.qw**2 - Q.qx**2 + Q.qy**2 - Q.qz**2
    tranform[1][2] = 2 * (Q.qy*Q.qz - Q.qw*Q.qx)
    tranform[2][0] = 2 * (Q.qx*Q.qz - Q.qw*Q.qy)
    tranform[2][1] = 2 * (Q.qy*Q.qz + Q.qw*Q.qx)
    tranform[2][2] = Q.qw**2 - Q.qx**2 - Q.qy**2 + Q.qz**2
    tranform[0][3] = D.x 
    tranform[1][3] = D.y 
    tranform[2][3] = D.z
    tranform[3] = [0,0,0,1]
    return tranform

def get_rotation_from_tranform(T):
    """Extract rotation matrix out from transform matrix."""
    rotation = np.zeros(shape=(3,3), dtype=float)
    rotation[0][0] = T[0][0]
    rotation[0][1] = T[0][1]
    rotation[0][2] = T[0][2]
    rotation[1][0] = T[1][0]
    rotation[1][1] = T[1][1]
    rotation[1][2] = T[1][2]
    rotation[2][0] = T[2][0]
    rotation[2][1] = T[2][1]
    rotation[2][2] = T[2][2]
    return rotation

def transform_coordinate(P, T):
    """Transform coordinate system according to rotation and translation."""
    point_mat = point3d_to_matrix(P)
    #point_rotation = np.matmul(T, point_mat)
    point_mat = np.dot(T, point_mat)
    P.x = point_mat[0][0]
    P.y = point_mat[1][0]
    P.z = point_mat[2][0]

def multiply_quaternion(Q1, Q2):
    """Multiple two quaternions. Q1 is the rotation applied AFTER Q2."""
    q = Quaternion()
    q.qw = Q1.qw*Q2.qw - Q1.qx*Q2.qx - Q1.qy*Q2.qy - Q1.qz*Q2.qz
    q.qx = Q1.qw*Q2.qx + Q1.qx*Q2.qw + Q1.qy*Q2.qz - Q1.qz*Q2.qy
    q.qy = Q1.qw*Q2.qy - Q1.qx*Q2.qy + Q1.qy*Q2.qw + Q1.qz*Q2.qx
    q.qz = Q1.qw*Q2.qz + Q1.qx*Q2.qy - Q1.qy*Q2.qx + Q1.qz*Q2.qw
    return q

def get_world_coordinate(T, pose):
    """Get world coordinate by using transform matrix (imu pose)"""
    pose_transform = generate_transform(pose._orientation, pose._position)
    T = np.dot(pose_transform, T)
    return T

def convert_to_world_coordinate(P, T, pose):
    """
    Convert to world coordinate by two steps:
    1. from imu to world by using transform matrix (imu pose)
    2. every point substract by the original point to match the visualizer
    """
    if pose is not None:
        T = get_world_coordinate(T, pose)
    transform_coordinate(P, T)
    P.x -= Sensor._stationary_pole[0]
    P.y -= Sensor._stationary_pole[1]
    P.z -= Sensor._stationary_pole[2]

def apply_scale_rotation(Q):
    """Apply additional Scale rotation."""
    QT = Quaternion()
    QT.qw = 0.5; QT.qx = -0.5; QT.qy = 0.5; QT.qz = -0.5
    return multiply_quaternion(QT, Q)

def check_stationary_pole(pose, is_primary):
    """Check if stationary pole is ready to use."""
    if pose._position is None:
        return False
    if Sensor._stationary_pole is None:
        if not is_primary:
            # Drop the points before primary lidar message comes
            return False
        Sensor._stationary_pole = (
            pose._position.x,
            pose._position.y,
            pose._position.z,
        )
    return True    

class Sensor(object):
    """Sensor class representing various of sensors."""
    # The original point as static coordinate frame of reference
    _stationary_pole = None
    def __init__(self, channel, intrinsics, extrinsics):
        """Constructor."""
        self._channel = channel
        self._intrinsics = load_yaml_settings(intrinsics)
        self._extrinsics = load_yaml_settings(extrinsics)
        self.initialize_transform()
        self._current = None
        self._current_list = []
        self._is_primary = False
        g_channel_process_map[self._channel] = self
 
    def process(self, message):
        """Processing function."""
        pass
    
    def construct_frame(self, frame):
        """Construct the current frame with sensor's data."""
        if len(self._current_list) != 0:
            self._current = self._current_list[-1]
            #self._current = deepcopy(self._current_list[-1])
        elif self._current is not None:
            self._current_list.append(self._current)
        self.offer_data(frame)
        self._current_list = []
    
    def offer_data(self, frame):
        """Processing function."""
        pass

    def clear_data(self):
        """Drop the data hold in the sensor"""
        self._current = None
        self._current_list = []

    def sensor_data_available(self):
        """Check if the sensor has data available."""
        return self._current is not None or len(self._current_list) != 0
    
    def initialize_transform(self):
        if self._extrinsics is None:
            return
        q = Quaternion()
        q.qw = self._extrinsics['transform']['rotation']['w']
        q.qx = self._extrinsics['transform']['rotation']['x']
        q.qy = self._extrinsics['transform']['rotation']['y']
        q.qz = self._extrinsics['transform']['rotation']['z']
        d = Point3D()
        d.x = self._extrinsics['transform']['translation']['x']
        d.y = self._extrinsics['transform']['translation']['y']
        d.z = self._extrinsics['transform']['translation']['z']
        self._transform = generate_transform(q, d)

    def add_transform(self, transform):
        self._transform = np.dot(transform, self._transform)

class PointCloudSensor(Sensor):
    """Lidar sensor that hold pointcloud data."""
    def __init__(self, channel, intrinsics, extrinsics, pose):
        """Initialization."""
        super(PointCloudSensor, self).__init__(channel, intrinsics, extrinsics)
        self._pose = pose

    def process(self, message):
        """Process PointCloud message."""
        if not check_stationary_pole(self._pose, self._is_primary):
            return
        point_cloud = PointCloud()
        point_cloud.ParseFromString(message)
        transform = get_world_coordinate(self._transform, self._pose)
        for point in point_cloud.point:
            convert_to_world_coordinate(point, transform, None)
            vector4 = frame_pb2.Vector4()
            vector4.x = point.x
            vector4.y = point.y
            vector4.z = point.z
            vector4.i = point.intensity
            self._current_list.append(vector4)

    def offer_data(self, frame):
        """Provide data to current frame."""
        for vector4 in self._current_list:
            point = frame.points.add()
            point.CopyFrom(vector4)
        if not self._is_primary:
            return
        point = Point3D()
        point.x = 0; point.y = 0; point.z = 0
        transform = get_world_coordinate(self._transform, self._pose)
        convert_to_world_coordinate(point, transform, None)
        frame.device_position.x = point.x
        frame.device_position.y = point.y
        frame.device_position.z = point.z
        rotation = get_rotation_from_tranform(transform)
        q = rotation_to_quaternion(rotation)
        # TODO: either apply it to all or do not apply it
        #q = apply_scale_rotation(q)
        frame.device_heading.x = q.qx
        frame.device_heading.y = q.qy
        frame.device_heading.z = q.qz
        frame.device_heading.w = q.qw

class RadarSensor(Sensor):
    """Radar sensor that hold radar data."""
    def __init__(self, channel, intrinsics, extrinsics, pose, transforms, type):
        """Initialization."""
        super(RadarSensor, self).__init__(channel, intrinsics, extrinsics)
        self._pose = pose
        self._accumulated_num = 300
        self._type = type
        for T in transforms:
            self.add_transform(T)

    def process(self, message):
        """Processing radar message."""
        if not check_stationary_pole(self._pose, self._is_primary):
            return
        radar = ContiRadar()
        radar.ParseFromString(message)
        transform = get_world_coordinate(self._transform, self._pose)
        for point in radar.contiobs:
            point3d = Point3D()
            point3d.x = point.longitude_dist
            point3d.y = point.lateral_dist
            point3d.z = 0
            convert_to_world_coordinate(point3d, transform, None)
            radar_point = frame_pb2.RadarPoint()
            radar_point.type = self._type 
            radar_point.position.x = point3d.x
            radar_point.position.y = point3d.y
            radar_point.position.z = point3d.z
            #radar_point.position.z = 0
            point3d.x = point.longitude_dist + point.longitude_vel
            point3d.y = point.lateral_dist + point.lateral_vel
            point3d.z = 0
            convert_to_world_coordinate(point3d, transform, None)
            radar_point.direction.x = point3d.x - radar_point.position.x
            radar_point.direction.y = point3d.y - radar_point.position.y
            radar_point.direction.z = point3d.z - radar_point.position.z
            #radar_point.direction.z = 0
            self._current_list.append(radar_point)
            if len(self._current_list) > self._accumulated_num:
                self._current_list.pop(0)
    
    def offer_data(self, frame):
        """Provide data to current frame."""
        for radar in self._current_list:
            new_radar = frame.radar_points.add()
            new_radar.CopyFrom(radar)

class ImageSensor(Sensor):
    """Image sensor that hold camera data."""
    # Image sequence number
    _image_seq = 0

    def __init__(self, channel, intrinsics, extrinsics, pose, path, url, \
        transforms):
        """Initialization."""
        super(ImageSensor, self).__init__(channel, intrinsics, extrinsics)
        self._output_dir = path
        self._image_url = url
        self._accumulated_num = 1
        self._pose = pose
        for T in transforms:
            self.add_transform(T)

    def process(self, message):
        """Processing image message."""
        if not check_stationary_pole(self._pose, self._is_primary):
            return
        image = CompressedImage()
        image.ParseFromString(message)
        camera_image = frame_pb2.CameraImage()
        camera_image.timestamp = image.header.timestamp_sec
        dump_img_bin(image.data, self._output_dir, ImageSensor._image_seq)
        camera_image.image_url = '{}/{}/pic-{}.jpg'.format(
            self._image_url, 
            os.path.basename(self._output_dir),
            ImageSensor._image_seq)
        ImageSensor._image_seq += 1
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
        point = Point3D()
        point.x = 0; point.y = 0; point.z = 0
        transform = get_world_coordinate(self._transform, self._pose)
        convert_to_world_coordinate(point, transform, None)
        camera_image.position.x = point.x
        camera_image.position.y = point.y
        camera_image.position.z = point.z
        rotation = get_rotation_from_tranform(transform)
        q = rotation_to_quaternion(rotation)
        # TODO: either apply it to all or do not apply it
        #q = apply_scale_rotation(q)
        camera_image.heading.x = q.qx
        camera_image.heading.y = q.qy
        camera_image.heading.z = q.qz
        camera_image.heading.w = q.qw
        camera_image.channel = self._channel
        self._current_list.append(camera_image)
        if len(self._current_list) > self._accumulated_num:
            self._current_list.pop(0)
    
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

    def offer_data(self, frame):
        """Provide data to current frame."""
        frame.device_gps_pose.CopyFrom(self._current)
        
class FramePopulator:
    """Extract sensors data from record file, and populate to JSON."""
    def __init__(self, args):
        self._args = args
        self._current_frame = frame_pb2.Frame()
        self._skipped_frame_count = 0
        self._frame_count = 0
        self.set_sensors(args)
        ImageSensor._image_seq = args.initial_image_seq
        self._frame_seq = args.initial_frame_seq
        if args.reset_stationary_pole:
            Sensor._stationary_pole = None
    
    def set_sensors(self, args):
        """Set all sensor instances."""
        self._gps_pose = GpsSensor(args.pose_channel, None, None)
        self._pointcloud_128 = PointCloudSensor(
            args.pointcloud_128_channel,
            None,
            args.pointcloud_128_extrinsics,
            self._gps_pose
        )
        self._pointcloud_128._is_primary = True

        # Current only use the 128 Lidar
        # self._pointcloud_16_front = PointCloudSensor(
        #    args.pointcloud_front_channel,
        #    None,
        #    args.pointcloud_front_extrinsics,
        #    self._gps_pose
        #)
        #self._pointcloud_16_rear_left = PointCloudSensor(
        #    args.pointcloud_rear_left_channel,
        #    None,
        #    args.pointcloud_rear_left_extrinsics,
        #    self._gps_pose
        #)
        #self._pointcloud_16_rear_right = PointCloudSensor(
        #    args.pointcloud_rear_right_channel,
        #    None,
        #    args.pointcloud_rear_right_extrinsics,
        #    self._gps_pose
        #)
        self._image_front_6mm = ImageSensor(
            args.image_front_6mm_channel,
            args.image_front_6mm_intrinsics,
            args.image_front_6mm_extrinsics,
            self._gps_pose,
            args.output_path,
            args.image_url_path,
            [self._pointcloud_128._transform])
        self._image_front_12mm = ImageSensor(
            args.image_front_12mm_channel,
            args.image_front_12mm_intrinsics,
            args.image_front_12mm_extrinsics,
            self._gps_pose,
            args.output_path,
            args.image_url_path,
            [self._pointcloud_128._transform])
        self._image_left_fisheye = ImageSensor(
            args.image_left_fisheye_channel,
            args.image_left_fisheye_intrinsics,
            args.image_left_fisheye_extrinsics,
            self._gps_pose,
            args.output_path,
            args.image_url_path,
            [self._pointcloud_128._transform])
        self._image_right_fisheye = ImageSensor(
            args.image_right_fisheye_channel,
            args.image_right_fisheye_intrinsics,
            args.image_right_fisheye_extrinsics,
            self._gps_pose,
            args.output_path,
            args.image_url_path,
            [self._pointcloud_128._transform])
        self._image_rear = ImageSensor(
            args.image_rear_6mm_channel,
            args.image_rear_6mm_intrinsics,
            args.image_rear_6mm_extrinsics,
            self._gps_pose,
            args.output_path,
            args.image_url_path,
            [self._pointcloud_128._transform])
        self._radar_front = RadarSensor(
            args.radar_front_channel,
            None,
            args.radar_front_extrinsics,
            self._gps_pose,
            [self._pointcloud_128._transform],
            frame_pb2.RadarPoint.FRONT
        )
        self._radar_rear = RadarSensor(
            args.radar_rear_channel,
            None,
            args.radar_rear_extrinsics,
            self._gps_pose,
            [self._pointcloud_128._transform],
            frame_pb2.RadarPoint.REAR
        )

    def construct_current_frame(self, message):
        """Construct the current frame to make it ready for dumping."""
        if self._gps_pose._position is None:
            return
        for sensor in g_channel_process_map.values():
            sensor.construct_frame(self._current_frame)
        self._frame_count += 1
        point_cloud = PointCloud()
        point_cloud.ParseFromString(message)
        self._current_frame.timestamp = point_cloud.header.timestamp_sec
        self._current_frame.frame_seq = \
            self._skipped_frame_count + self._frame_count
        self._current_frame.data_file = os.path.basename(self._args.input_file)
        glog.info('#dumping frame {:d}: {:.7f}'.format(
		self._frame_count, self._current_frame.timestamp))
        self.dump_to_json_file()
        self._current_frame = frame_pb2.Frame()

    def check_all_sensors_data(self):
        """Ensure all sensors have data"""
        for sensor in g_channel_process_map.values():
            if not sensor.sensor_data_available():
                return False
        return True

    def dump_to_json_file(self):
        """Dump the frame content to JSON file."""
        file_name = os.path.join(self._args.output_path,'frame-{}.json'.format(
            self._args.initial_frame_seq + self._frame_count))
        jsonObj = MessageToJson(self._current_frame, False, True)
        with open(file_name, 'w') as outfile:
            outfile.write(jsonObj)
     
    def process_record_file(self):
        """Read record file and extract the message with specified channels"""
        freader = record.RecordReader(self._args.input_file)
        time.sleep(1)
        glog.info('#processing record file {}'.format(self._args.input_file))
        for channel, message, _type, _timestamp in freader.read_messages():
            if self._frame_count >= self._args.maximum_frame_count:
                glog.info('#reached the maximum frame count, exiting now')
                return self._args.maximum_frame_count
            if channel in g_channel_process_map:
                if self._skipped_frame_count < self._args.skip_frame_count:
                    if g_channel_process_map[channel]._is_primary:
                        self._skipped_frame_count += 1
                    continue
                g_channel_process_map[channel].process(message)
                if g_channel_process_map[channel]._is_primary:
                    if not self.check_all_sensors_data():
                        g_channel_process_map[channel].clear_data()
                        self._skipped_frame_count += 1
                        continue
                    self.construct_current_frame(message)
        return self._frame_count

def main():
    """Entry point."""
    gflags.FLAGS(sys.argv)
    frame_populator = FramePopulator(gflags.FLAGS)
    frame_populator.process_record_file()
    return

if __name__ == '__main__':
    main()

