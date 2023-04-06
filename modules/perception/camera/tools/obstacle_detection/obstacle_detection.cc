/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <string>
#include <vector>

#include "gflags/gflags.h"

#include "modules/perception/pipeline/proto/pipeline_config.pb.h"

#include "cyber/common/file.h"
#include "modules/perception/camera/app/obstacle_detection_camera.h"
#include "modules/perception/camera/tools/common/ground_truth.h"
#include "modules/perception/camera/tools/common/util.h"
#include "modules/perception/camera/tools/common/visualizer.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/pipeline/data_frame.h"


DEFINE_int32(height, 1080, "image height");
DEFINE_int32(width, 1920, "image width");
DEFINE_string(dest_dir, "./data", "output dir");
DEFINE_string(dist_type, "", "dist pred type: H-on-h, H-from-h");
DEFINE_string(kitti_dir, "", "pre-detected obstacles (skip Detect)");
DEFINE_string(root_dir, "/apollo/modules/perception/camera/tools/obstacle_detection", "image root dir"); // NOLINT
DEFINE_string(image_ext, ".jpg", "extension of image name");
DEFINE_string(test_list, "/apollo/modules/perception/camera/tools/obstacle_detection/images/image_test_list.txt", "test image list"); // NOLINT
DEFINE_string(obstacle_detection_conf_file, "/apollo/modules/perception/camera/tools/obstacle_detection/conf/camera_detection_pipeline.pb.txt", "Camera perception config file"); // NOLINT
DEFINE_string(camera_intrinsics, "/apollo/modules/perception/camera/tools/obstacle_detection/params/onsemi_obstacle_intrinsics.yaml", "Camera intrinsic file"); // NOLINT

namespace apollo {
namespace perception {
namespace camera {

bool InitDataProvider(CameraFrame* camera_frame) {
  DataProvider::InitOptions init_options;
  init_options.sensor_name = "front_6mm";
  init_options.image_height = FLAGS_height;
  init_options.image_width = FLAGS_width;
  init_options.device_id = FLAGS_gpu_id;

  bool res = camera_frame->data_provider->Init(init_options);
  return res;
}

void TestDetection() {
  // Init frame
  pipeline::DataFrame frame;
  CameraFrame camera_frame;
  DataProvider data_provider;
  frame.camera_frame = &camera_frame;
  camera_frame.data_provider = &data_provider;
  camera_frame.track_feature_blob.reset(new base::Blob<float>());

  // Init data
  ACHECK(InitDataProvider(&camera_frame));
  ACHECK(FillCameraKMatrix(&camera_frame, FLAGS_camera_intrinsics));

  // Init detection pipeline
  ObstacleDetectionCamera detection_pipeline;
  pipeline::PipelineConfig camera_obstacle_detection_config;
  ACHECK(cyber::common::GetProtoFromFile(FLAGS_obstacle_detection_conf_file,
                                         &camera_obstacle_detection_config));
  ACHECK(detection_pipeline.Init(camera_obstacle_detection_config));

  // Load image list
  std::vector<std::string> file_list;
  ACHECK(GetFileListFromFile(FLAGS_test_list, &file_list));

  // Main process
  for (const auto& image_name : file_list) {
    AINFO << "image: " << image_name;

    if (FLAGS_kitti_dir != "") {
      std::string kitti_path = FLAGS_kitti_dir + "/" + image_name + ".txt";

      if (!LoadKittiLabel(&camera_frame, kitti_path, FLAGS_dist_type)) {
        AERROR << "Loading kitti result failed: " << kitti_path;
        continue;
      }
    } else {
      std::string image_path =
          FLAGS_root_dir + "/images/" + image_name + FLAGS_image_ext;

      ACHECK(FillImage(&camera_frame, image_path));
      ACHECK(detection_pipeline.Process(&frame));
    }

    // Results visualization
    std::string result_path = FLAGS_dest_dir + "/" + image_name + ".txt";
    ACHECK(SaveCameraDetectionResult(&camera_frame, result_path));

    result_path = FLAGS_dest_dir + "/" + image_name + ".jpg";
    ACHECK(CameraVisualization(&camera_frame, result_path));
  }
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo

int main(int argc, char *argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::SetUsageMessage(
      "command line brew\n"
      "Usage: camera_benchmark <args>\n");
  apollo::perception::camera::TestDetection();
  return 0;
}
