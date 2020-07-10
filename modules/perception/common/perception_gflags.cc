/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/perception/common/perception_gflags.h"

#include <climits>

namespace apollo {
namespace perception {

// sensor_manager
DEFINE_string(obs_sensor_intrinsic_path,
              "/apollo/modules/perception/data/params",
              "The intrinsics/extrinsics dir.");

DEFINE_string(obs_sensor_meta_path,
              "/apollo/modules/perception/production"
              "/data/perception/common/sensor_meta.pt",
              "The SensorManager config file.");

DEFINE_bool(enable_base_object_pool, false, "Enable base object pool.");

// config_manager
DEFINE_string(config_manager_path, "./conf", "The ModelConfig config paths.");
DEFINE_string(work_root, "", "Project work root direcotry.");

// lidar_point_pillars
DEFINE_int32(gpu_id, 0, "The id of gpu used for inference.");
DEFINE_string(pfe_onnx_file,
              "/apollo/modules/perception/production/data/perception/lidar/"
              "models/detection/point_pillars/pfe.onnx",
              "The path of pillars feature extractor onnx file.");
DEFINE_string(rpn_onnx_file,
              "/apollo/modules/perception/production/data/perception/lidar/"
              "models/detection/point_pillars/rpn.onnx",
              "The path of RPN onnx file.");
DEFINE_double(normalizing_factor, 255,
              "Normalize intensity range to [0, 1] by this factor.");
DEFINE_int32(num_point_feature, 5,
             "Length of raw point feature. Features include x, y, z,"
             "intensity and delta of time.");
DEFINE_bool(enable_downsample_beams, false,
            "Enable down sampling point cloud beams with a factor.");
DEFINE_int32(downsample_beams_factor, 4,
             "Down sample point cloud beams with this factor.");
DEFINE_bool(enable_downsample_pointcloud, false,
            "Enable down sampling point cloud into centroids of voxel grid.");
DEFINE_double(downsample_voxel_size_x, 0.01,
              "X-axis size of voxels used for down sampling point cloud.");
DEFINE_double(downsample_voxel_size_y, 0.01,
              "Y-axis size of voxels used for down sampling point cloud.");
DEFINE_double(downsample_voxel_size_z, 0.01,
              "Z-axis size of voxels used for down sampling point cloud.");
DEFINE_bool(enable_fuse_frames, false,
            "Enable fusing preceding frames' point cloud into current frame.");
DEFINE_int32(num_fuse_frames, 5,
             "Number of frames to fuse, including current frame.");
DEFINE_double(fuse_time_interval, 0.5,
              "Time interval in seconds of frames to fuse.");
DEFINE_bool(enable_shuffle_points, false,
            "Enable shuffling points before preprocessing.");
DEFINE_int32(max_num_points, INT_MAX, "Max number of points to preprocess.");
DEFINE_bool(reproduce_result_mode, false, "True if preprocess in CPU mode.");
DEFINE_double(score_threshold, 0.5, "Classification score threshold.");
DEFINE_double(nms_overlap_threshold, 0.5, "Nms overlap threshold.");
DEFINE_int32(num_output_box_feature, 7, "Length of output box feature.");

}  // namespace perception
}  // namespace apollo
