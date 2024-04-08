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

DEFINE_string(obs_sensor_meta_file, "sensor_meta.pb.txt",
              "The SensorManager config file.");

DEFINE_bool(enable_base_object_pool, false, "Enable base object pool.");

// config_manager
DEFINE_string(config_manager_path, "./", "The ModelConfig config paths.");
DEFINE_string(work_root, "/apollo/modules/perception",
              "Project work root direcotry.");

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

DEFINE_string(object_template_file, "object_template.pb.txt",
              "object template config file");

DEFINE_int32(hdmap_sample_step, 1, "hdmap sample step");

// lidar_center_point
DEFINE_bool(use_trt, false, "True if preprocess in CPU mode.");
DEFINE_int32(trt_precision, 1,
             "Precision type of tensorrt, 0: kFloat32, 1: kInt8, 2: kHalf");
DEFINE_int32(
    trt_use_static, 1,
    "Whether to load the tensorrt graph optimization from a disk path");
DEFINE_bool(use_calibration, true, "Whether to use calibration table");
DEFINE_bool(collect_shape_info, false,
            "Whether to collect dynamic shape before using tensorrt");
DEFINE_bool(use_dynamicshape, true,
            "Whether to use dynamic shape when using tensorrt");
DEFINE_string(dynamic_shape_file,
              "/apollo/modules/perception/data/models/"
              "center_point_paddle/collect_shape_info_3lidar_20.pbtxt",
              "Path of a dynamic shape file for tensorrt");

// scene manager
DEFINE_string(scene_manager_file, "scene_manager.conf",
              "scene manager config file");
DEFINE_string(roi_service_file, "roi_service.conf", "roi service config file");
DEFINE_string(ground_service_file, "ground_service.conf",
              "ground service config file");

}  // namespace perception
}  // namespace apollo
