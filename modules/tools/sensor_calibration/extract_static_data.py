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
This is a tool to etract useful information from already extracted sensor data,
mainly for camera lidar calibration.
"""

from datetime import datetime
import argparse
import os
import sys
import shutil
import six
import numpy as np
import cv2
from shutil import copyfile
from google.protobuf import text_format

from cyber_py.record import RecordReader
from cyber.proto import record_pb2
from modules.tools.sensor_calibration.proto import extractor_config_pb2

from sensor_msg_extractor import *
from data_file_object import *


CYBER_PATH = os.environ['CYBER_PATH']
CYBER_RECORD_HEADER_LENGTH = 2048

def mkdir_p(path):
    if not os.path.isdir(path):
        os.makedirs(path)


def sort_files_by_timestamp(in_path, out_path,
                            timestamp_filename,
                            extension='.png'):
    """sort files by timestamp"""
    ts_file = os.path.join(in_path, timestamp_filename)
    out_ts_file = os.path.join(out_path, timestamp_filename)
    ts_map = np.loadtxt(ts_file)
    sorted_ids = np.argsort(ts_map[:,1])
    ts_map = ts_map[sorted_ids]
    # sorted_ts = np.vstack((np.arange(sorted_ids), ts_map[:,1])).T
    # np.savetxt(out_ts_file, ts_map, sorted_ts)
    ts_obj = TimestampFileObject(file_path=out_ts_file);
    ts_obj.save_to_file(ts_map[:,1])

    if extension=='.png' or extension=='.pcd':
        for i, idx in enumerate(ts_map[:,0]):
            in_file_name = os.path.join(in_path, ("%06d"%(idx+1)) + extension)
            out_file_name = os.path.join(out_path, ("%06d"%(i+1)) + extension)
            copyfile(in_file_name, out_file_name)

    elif extension=='Odometry.bin':
        tmp_file = os.path.join(in_path, 'Odometry.bin')
        in_odm = OdometryFileObject(file_path=tmp_file,
                                    operation='read',
                                    file_type='binary')
        data = in_odm.load_file()
        sorted_data = []
        for idx in ts_map[:, 0]:
            d = data[idx]
            sorted_data.append(d)

        tmp_file = os.path.join(out_path, 'Odometry.bin')
        out_odm = OdometryFileObject(file_path=tmp_file,
                                    operation='write',
                                    file_type='binary')
        out_odm.save_to_file(sorted_data)

def find_nearest(array,value):
    idx = np.searchsorted(array, value, side="left")
    if idx > 0 and (idx == len(array) or \
        math.fabs(value - array[idx-1]) < math.fabs(value - array[idx])):
        return idx-1
    else:
        return idx

def find_synchronized_timestamp_index(query_ts, source_ts, max_threshold=1.0):
    # for each element in query, find the nearest element in source,
    # if the distance < max_threshold
    ts1 = query_ts
    ts2 = source_ts
    if len(ts1) == 0 or len(ts2) == 0:
        return {}

    nearest_index_map = {}
    start = 0
    for i in range(len(ts1)):
        if start >= len(ts2):
            break

        min_ts_diff = max_threshold
        min_j = -1
        for j in range(start, len(ts2)):
            ts_diff = ts2[j] - ts1[i]
            if abs(ts_diff) < min_ts_diff:
                min_j = j
                min_ts_diff = abs(ts_diff)

            if ts_diff >= 0:
               # the rest ts2 > ts1, no need to loop ts2 anymore
               break

        if min_j > 0: #find valid nearest index in ts2
            start = min_j # reset start for case i++
            nearest_index_map[i] = min_j
    # for i, j in nearest_index_map.items():
    #     print([i, j, query_ts[i], source_ts[j]])
    return nearest_index_map

def get_difference_score_between_images(path, file_indexes,
                                        suffix=".png", thumbnail_size=32):
    image_sum = np.zeros(len(file_indexes), dtype=np.float32)
    image_diff = np.zeros(len(file_indexes), dtype=np.float32)
    image_thumbnails = []
    for c, idx in enumerate(file_indexes):
        image_file = os.path.join(path, str(int(idx)).zfill(6)+suffix)
        image = cv2.imread(image_file)
        image_thumbnails.append(cv2.resize(image,
            (thumbnail_size, thumbnail_size),interpolation=cv2.INTER_AREA))
        image_sum[c] =  np.sum(image_thumbnails[-1])

    image_diff[0] = np.finfo(float).max
    for c in range(len(file_indexes)-1, 0, -1):
        image_diff[c] = np.sum(cv2.absdiff(image_thumbnails[c], image_thumbnails[c-1]))
        image_diff[c] = image_diff[c] / image_sum[c]

    # print("image_diff is: ")
    # for i in range(len(image_diff)):
    #     print([i, image_diff[i], image_sum[i]])
    return image_diff

def get_distance_by_odometry(data, i, j):
    #  calculate x-y plane distance
    return np.linalg.norm(data[i,-3:-1] - data[j,-3:-1])

def check_static_by_odometry(data, index, check_range=40,
                            movable_threshold=0.01):
    start_idx = np.maximum(index - check_range, 0)
    end_idx = np.minimum(index + check_range, data.shape[0]-1)
    #  skip if start and end index are too nearby.
    if end_idx - start_idx <= check_range:
        return False
    #  calculate x-y plane distance
    distance = get_distance_by_odometry(data, start_idx, end_idx)
    # print("distance is %d %d %f" % (start_idx, end_idx,distance))
    return distance < movable_threshold

def select_static_image_pcd(path, min_distance=5, stop_times=5,
                            wait_time=3, check_range=50,
                            image_static_diff_threshold=0.005,
                            image_suffix='.png', pcd_suffix='.pcd'):
    """select pairs of images and pcds"""
    subfolders = [x[0] for x in os.walk(path) if '_apollo_' in x[0] \
                    and 'camera-lidar-pairs' not in x[0]]
    lidar_subfolder = [x for x in subfolders if '_lidar' in x]
    odometry_subfolder = [x for x in subfolders if'_odometry' in x]
    camera_subfolders = [x for x in subfolders if'_camera' in x]
    if len(lidar_subfolder) is not 1 or \
        len(odometry_subfolder) is not 1:
        raise ValueError("only one main lidar and one Odometry \
                        sensor is needed for sensor calibration")

    lidar_subfolder = lidar_subfolder[0]
    odometry_subfolder = odometry_subfolder[0]
    #  load timestamp dictionary
    timestamp_dict = {}
    for f in subfolders:
        f_abs_path = os.path.join(path, f)
        ts_file = os.path.join(f_abs_path, 'timestamps.txt')
        ts_map = np.loadtxt(ts_file)
        timestamp_dict[f] = ts_map
    #  load odometry binary file
    odometry_file = os.path.join(path, odometry_subfolder, 'Odometry.bin')
    in_odm = OdometryFileObject(file_path=odometry_file,
                      operation='read',
                      file_type='binary')
    odometry_data = np.array(in_odm.load_file())
    for camera in camera_subfolders:
        print("working on sensor message: {}".format(camera))
        camera_gps_nearest_pairs = \
            find_synchronized_timestamp_index(
                timestamp_dict[camera][:,1],
                timestamp_dict[odometry_subfolder][:,1],
                max_threshold=0.2)

        camera_lidar_nearest_pairs = \
            find_synchronized_timestamp_index(
                timestamp_dict[camera][:,1],
                timestamp_dict[lidar_subfolder][:,1],
                max_threshold=0.5)

         # clean camera-lidar paris not exist in both dictionary
        for key in camera_lidar_nearest_pairs:
            if key not in camera_gps_nearest_pairs:
                del camera_lidar_nearest_pairs[key]

        camera_folder_path = os.path.join(path, camera)
        camera_diff = get_difference_score_between_images(
            camera_folder_path, timestamp_dict[camera][:,0])
        valid_image_indexes =  [x for x, v in enumerate(camera_diff) \
                                if v <= image_static_diff_threshold]
        valid_images = (timestamp_dict[camera][valid_image_indexes, 0]).astype(int)
        # generate valid camera frame
        candidate_idx = []
        last_idx = -1
        last_odometry_idx = -1
        for i in valid_images:
            if i in camera_lidar_nearest_pairs:
                odometry_idx = camera_gps_nearest_pairs[i]
                #  not static considering odometry motion.
                if not check_static_by_odometry(odometry_data,
                    odometry_idx, check_range=check_range):
                    continue

                if last_idx is -1:
                    last_idx = i
                    last_odometry_idx = odometry_idx
                    continue
                time_interval = timestamp_dict[camera][i,1] - timestamp_dict[camera][last_idx, 1]
                odomerty_interval = \
                    get_distance_by_odometry(odometry_data, odometry_idx, last_odometry_idx)
                #timestamp interval > wait_time and odometry_interval > min_distance`
                if time_interval < wait_time or \
                    odomerty_interval < min_distance:
                    continue

                candidate_idx.append(i)
                last_idx = i
                last_odometry_idx = odometry_idx
                # print(odometry_data[odometry_idx])
                # print(odometry_data[last_odometry_idx])
                # print([i, odomerty_interval])
        #  check candidate number and select best stop according to camera_diff score
        print("all valid static image index: ", candidate_idx)
        if len(candidate_idx) < stop_times:
            raise ValueError("not enough stops detected, \
                thus no sufficient data for camera-lidar calibration")
        elif len(candidate_idx) > stop_times:
            tmp_diff = camera_diff[candidate_idx]

            tmp_idx = np.argsort(tmp_diff)[:stop_times]
            candidate_idx = [candidate_idx[x] for x in tmp_idx]
            candidate_idx.sort()
        #  save files
        image_idx = candidate_idx
        print("selected best static image index: ", image_idx)
        lidar_idx = [ camera_lidar_nearest_pairs[x] for x in image_idx ]
        output_path = os.path.join(camera_folder_path, 'camera-lidar-pairs')
        mkdir_p(output_path)
        for count, i in enumerate(image_idx):
            #  save images
            in_file = os.path.join(camera_folder_path, str(int(i)).zfill(6)+image_suffix)
            out_file = os.path.join(output_path, str(int(count)).zfill(6)+image_suffix)
            copyfile(in_file, out_file)
            j = camera_lidar_nearest_pairs[i]
            #  save pcd
            in_file = os.path.join(path, lidar_subfolder, str(int(j)).zfill(6)+pcd_suffix)
            out_file = os.path.join(output_path, str(int(count)).zfill(6)+pcd_suffix)
            copyfile(in_file, out_file)
            print("generate image-lidar-pair:[%d, %d]" % (i, j))

"""
    # load odometry
    odometry_file = os.path.join(path, odometry_subfolder, 'Odometry.bin')
    in_odm = Odometry(file_path=odometry_file, operation='read', file_type='binary')
    data = in_odm.load_file()
    select_ts_array = np.zeros((5,), datatype=float64)
    count = 0
    last_ts = data[0,:]

    for i in range(check_range, data.shape[0]-check_range):
        cur_ts = data[i,:]
        if cur_ts[0] - last_ts[0] > wait_time:
            dis = np.linalg.norm(cur_ts[-3:], last_ts[-3,:])
            if dis > min_distance:
                cur_id = i
                start_id = i - check_range
                end_id = i + check_range
                dist_range = np.linalg.norm(data[start_id, -3:],
                                            data[end_id, -3:])
                if dist_range < 0.05: # static based on GPS
                    # 1. find closest image time
                    # 2. check if image static
                    # if not, move image timestamp?
                    # 3. after find image static, search nearest pcd
                    # move the pairs to a folder, created inside each camera folder?
                    # 4. done for this camera, move to next camera.
                    for camera in camera_subfolders:
                        camera_path = os.path.join(path, camera)
                        camera_ts = timestamp_dict[camera]
                        idx = find_nearsest(camera_ts[:,1], cur_ts[0])
                        if np.abs(camera_ts[idx,1] - cur_ts[0]) < 0.06: # suppose 20HZ
"""

def main2():
    for str in ['_apollo_sensor_lidar16_front_center_PointCloud2',
                '_apollo_sensor_lidar16_rear_left_PointCloud2',
                '_apollo_sensor_lidar16_rear_right_PointCloud2',
                '_apollo_sensor_lidar128_PointCloud2']:
         #_apollo_sensor_lidar128_PointCloud2'
        in_path = '/apollo/data/Lidar_GNSS_Calibration-2019-07-12-15-25/' + str
        out_path = os.path.join(in_path, 'new')
        mkdir_p(out_path)
        timestamp_filename = 'timestamps.txt'
        extension = '.pcd'
        sort_files_by_timestamp(in_path, out_path, timestamp_filename, extension=extension)

    in_path = '/apollo/data/Lidar_GNSS_Calibration-2019-07-12-15-25/_apollo_sensor_gnss_odometry'
    out_path = os.path.join(in_path, 'new')
    mkdir_p(out_path)
    timestamp_filename = 'timestamps.txt'
    extension = 'Odometry.bin'
    sort_files_by_timestamp(in_path, out_path, timestamp_filename, extension=extension)

def main():
    if CYBER_PATH is None:
     print('Error: environment variable CYBER_PATH was not found, '
           'set environment first.')
     sys.exit(1)

    os.chdir(CYBER_PATH)

    parser = argparse.ArgumentParser(
        description='A tool to extract useful data information for lidar-to-camera calibration.')
    parser.add_argument("-i", "--workspace_path", action="store", default="", required=True,
                        dest='workspace',
                        help="Specify the worksapce where storing extracted sensor messages")
    args = parser.parse_args()

    select_static_image_pcd(path=args.workspace, min_distance=5, stop_times=5,
                            wait_time=3, check_range=50,
                            image_static_diff_threshold=0.005,
                            image_suffix='.png', pcd_suffix='.pcd')

if __name__ == '__main__':
    main()
