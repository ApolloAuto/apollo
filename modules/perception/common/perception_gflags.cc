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

#include <limits>

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
DEFINE_string(work_root, "/apollo/modules/perception/production/",
              "Project work root direcotry.");

// lidar_point_pillars
DEFINE_int32(gpu_id, 0, "The id of gpu used for inference.");
DEFINE_string(pfe_torch_file,
              "/apollo/modules/perception/production/data/perception/lidar/"
              "models/detection/point_pillars_torch/pts_voxel_encoder.zip",
              "The path of pillars feature extractor torch file.");
DEFINE_string(scattered_torch_file,
              "/apollo/modules/perception/production/data/perception/lidar/"
              "models/detection/point_pillars_torch/pts_middle_encoder.zip",
              "The path of pillars feature scatter torch file.");
DEFINE_string(backbone_torch_file,
              "/apollo/modules/perception/production/data/perception/lidar/"
              "models/detection/point_pillars_torch/pts_backbone.zip",
              "The path of pillars backbone torch file.");
DEFINE_string(fpn_torch_file,
              "/apollo/modules/perception/production/data/perception/lidar/"
              "models/detection/point_pillars_torch/pts_neck.zip",
              "The path of pillars fpn torch file.");
DEFINE_string(bbox_head_torch_file,
              "/apollo/modules/perception/production/data/perception/lidar/"
              "models/detection/point_pillars_torch/pts_bbox_head.zip",
              "The path of pillars bbox head torch file.");
DEFINE_double(normalizing_factor, 255,
              "Normalize intensity range to [0, 1] by this factor.");
DEFINE_int32(num_point_feature, 5,
             "Length of raw point feature. Features include x, y, z,"
             "intensity and delta of time.");
DEFINE_bool(enable_ground_removal, false, "Enable ground removal.");
DEFINE_double(ground_removal_height, -1.5, "Height for ground removal.");
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
DEFINE_int32(max_num_points, std::numeric_limits<int>::max(),
             "Max number of points to preprocess.");
DEFINE_bool(reproduce_result_mode, false, "True if preprocess in CPU mode.");
DEFINE_double(score_threshold, 0.5, "Classification score threshold.");
DEFINE_double(nms_overlap_threshold, 0.5, "Nms overlap threshold.");
DEFINE_int32(num_output_box_feature, 7, "Length of output box feature.");

// lidar_mask_pillars
DEFINE_string(mask_pfe_torch_file,
              "/apollo/modules/perception/production/data/perception/lidar/"
              "models/detection/mask_pillars_torch/pts_voxel_encoder.zip",
              "The path of pillars feature extractor torch file.");
DEFINE_string(mask_scattered_torch_file,
              "/apollo/modules/perception/production/data/perception/lidar/"
              "models/detection/mask_pillars_torch/pts_middle_encoder.zip",
              "The path of pillars feature scatter torch file.");
DEFINE_string(mask_backbone_torch_file,
              "/apollo/modules/perception/production/data/perception/lidar/"
              "models/detection/mask_pillars_torch/pts_backbone.zip",
              "The path of pillars backbone torch file.");
DEFINE_string(mask_fpn_torch_file,
              "/apollo/modules/perception/production/data/perception/lidar/"
              "models/detection/mask_pillars_torch/pts_neck.zip",
              "The path of pillars fpn torch file.");
DEFINE_string(mask_bbox_head_torch_file,
              "/apollo/modules/perception/production/data/perception/lidar/"
              "models/detection/mask_pillars_torch/pts_bbox_head.zip",
              "The path of pillars bbox head torch file.");

// lidar_center_point
DEFINE_string(center_point_model_file,
              "/apollo/modules/perception/production/data/perception/lidar/"
              "models/detection/CenterPoint_paddle/centerpoint.pdmodel",
              "The path of center point model file.");
DEFINE_string(center_point_params_file,
              "/apollo/modules/perception/production/data/perception/lidar/"
              "models/detection/CenterPoint_paddle/centerpoint.pdiparams",
              "The path of center point params file.");
DEFINE_bool(use_trt, false, "True if preprocess in CPU mode.");
DEFINE_int32(trt_precision, 1,
             "Precision type of tensorrt, 0: kFloat32, 1: kHalf");
DEFINE_int32(
    trt_use_static, 0,
    "Whether to load the tensorrt graph optimization from a disk path");
DEFINE_string(
    trt_static_dir,
    "/apollo/modules/perception/lidar/lib/detector/center_point_detection/",
    "Path of a tensorrt graph optimization directory");
DEFINE_int32(collect_shape_info, 1,
             "Whether to collect dynamic shape before using tensorrt");
DEFINE_string(dynamic_shape_file,
              "/apollo/modules/perception/production/data/perception/lidar/"
              "models/detection/CenterPoint_paddle/collect_shape_info.pbtxt",
              "Path of a dynamic shape file for tensorrt");

// bev petr_v1
DEFINE_string(bev_model_file,
              "/apollo/modules/perception/production/data/perception/camera/"
              "models/petr_v1/petr_inference.pdmodel",
              "The path of bev model file.");
DEFINE_string(bev_params_file,
              "/apollo/modules/perception/production/data/perception/camera/"
              "models/petr_v1/petr_inference.pdiparams",
              "The path of bev params file.");

// caddn
DEFINE_string(caddn_model_file,
              "/apollo/modules/perception/production/data/perception/camera/"
              "models/caddn/model.pdmodel",
              "The path of caddn model file.");
DEFINE_string(caddn_params_file,
              "/apollo/modules/perception/production/data/perception/camera/"
              "models/caddn/model.pdiparams",
              "The path of caddn params file.");

// emergency detection onnx
DEFINE_string(onnx_obstacle_detector_model,
              "/apollo/modules/perception/camera"
              "/lib/obstacle/detector/yolov4/model/yolov4_1_3_416_416.onnx",
              "The onnx model file for emergency detection");
DEFINE_string(onnx_test_input_path,
              "/apollo/modules/perception/inference"
              "/onnx/testdata/dog.jpg",
              "The test input image file for onnx inference");
DEFINE_string(onnx_test_input_name_file,
              "/apollo/modules/perception/inference"
              "/onnx/testdata/coco.names",
              "The test input coco name file for onnx inference");
DEFINE_string(onnx_prediction_image_path,
              "/apollo/modules/perception/inference"
              "/onnx/testdata/prediction.jpg",
              "The prediction output image file for onnx inference");
DEFINE_int32(num_classes, 80, "number of classes for onnx inference");

// emergency detection libtorch
DEFINE_string(torch_detector_model,
              "/apollo/modules/perception/camera"
              "/lib/obstacle/detector/yolov4/model/yolov4.pt",
              "The torch model file for emergency detection");

// lidar sensor name
DEFINE_string(lidar_sensor_name, "velodyne128", "lidar sensor name");
}  // namespace perception
}  // namespace apollo
