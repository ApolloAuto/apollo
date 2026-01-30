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

#include <memory>
#include <string>
#include <vector>

#include "gflags/gflags.h"

#include "cyber/common/file.h"
#include "modules/perception/common/algorithm/io/io_util.h"
#include "modules/perception/common/algorithm/sensor_manager/sensor_manager.h"
#include "modules/perception/common/camera/common/data_provider.h"
#include "modules/perception/common/interface/base_obstacle_detector.h"
#include "modules/perception/common/onboard/inner_component_messages/camera_detection_component_messages.h"
#include "modules/perception/tools/common/ground_truth.h"
#include "modules/perception/tools/common/util.h"
#include "modules/perception/tools/common/visualizer.h"

DEFINE_int32(height, 1080, "image height");
DEFINE_int32(width, 1920, "image width");
DEFINE_int32(gpu_id, 0, "gpu id");
DEFINE_string(dest_dir, "./data/output", "output dir");
DEFINE_string(dist_type, "", "dist pred type: H-on-h, H-from-h");
DEFINE_string(kitti_dir, "", "pre-detected obstacles (skip Detect)");
DEFINE_string(root_dir,
              "/apollo/modules/perception/tools/offline_camera_detection/",
              "image root dir");
DEFINE_string(image_ext, ".jpg", "extension of image name");
DEFINE_string(config_path, "perception/camera_detection_multi_stage/data",
              "config path");
DEFINE_string(config_file, "yolox3d.pb.txt", "config file");
DEFINE_string(camera_name, "front_6mm", "camera name");
DEFINE_string(detector_name, "Yolox3DObstacleDetector", "detector name");

namespace apollo {
namespace perception {
namespace camera {

bool InitDataProvider(onboard::CameraFrame* camera_frame) {
  DataProvider::InitOptions init_options;
  init_options.sensor_name = FLAGS_camera_name;
  init_options.image_height = FLAGS_height;
  init_options.image_width = FLAGS_width;
  init_options.device_id = FLAGS_gpu_id;

  bool res = camera_frame->data_provider->Init(init_options);
  return res;
}

bool TestDetection() {
  // Init frame
  onboard::CameraFrame camera_frame;
  camera_frame.data_provider.reset(new DataProvider());
  camera_frame.feature_blob.reset(new base::Blob<float>());

  // Init data
  ACHECK(InitDataProvider(&camera_frame));

  ObstacleDetectorInitOptions init_options;
  // Init conf file
  init_options.config_path = FLAGS_config_path;
  init_options.config_file = FLAGS_config_file;
  init_options.gpu_id = FLAGS_gpu_id;

  // Init camera params
  std::string camera_name = FLAGS_camera_name;
  base::BaseCameraModelPtr model =
      algorithm::SensorManager::Instance()->GetUndistortCameraModel(
          camera_name);
  ACHECK(model) << "Can't find " << camera_name
                << " in data/conf/sensor_meta.pb.txt";
  auto pinhole = static_cast<base::PinholeCameraModel*>(model.get());
  init_options.intrinsic = pinhole->get_intrinsic_params();
  camera_frame.camera_k_matrix = pinhole->get_intrinsic_params();
  init_options.image_height = model->get_height();
  init_options.image_width = model->get_width();
  // Init detection pipeline
  std::shared_ptr<BaseObstacleDetector> detector;
  detector.reset(
      BaseObstacleDetectorRegisterer::GetInstanceByName(FLAGS_detector_name));
  detector->Init(init_options);

  // Load image list
  std::string images_path = FLAGS_root_dir + "/images/";
  std::vector<std::string> img_file_names;
  if (!algorithm::GetFileList(images_path, ".jpg", &img_file_names)) {
    AERROR << "images_path: " << images_path << " get file list error.";
    return false;
  }

  std::sort(img_file_names.begin(), img_file_names.end(),
            [](const std::string& lhs, const std::string& rhs) {
              if (lhs.length() != rhs.length()) {
                return lhs.length() < rhs.length();
              }
              return lhs <= rhs;
            });

  // Main process
  for (const auto& img_file : img_file_names) {
    AINFO << "img_file: " << img_file;

    std::string image_name = cyber::common::GetFileName(img_file, true);

    if (FLAGS_kitti_dir != "") {
      std::string kitti_path = FLAGS_kitti_dir + "/" + image_name + ".txt";

      if (!LoadKittiLabel(&camera_frame, kitti_path, FLAGS_dist_type)) {
        AERROR << "Loading kitti result failed: " << kitti_path;
        continue;
      }
    } else {
      ACHECK(FillImage(&camera_frame, img_file));
      ACHECK(detector->Detect(&camera_frame));
    }

    // Results visualization
    if (!cyber::common::EnsureDirectory(FLAGS_dest_dir)) {
      AERROR << "Failed to create: " << FLAGS_dest_dir;
      return false;
    }

    std::string result_path = FLAGS_dest_dir + "/" + image_name + ".txt";
    ACHECK(SaveCameraDetectionResult(&camera_frame, result_path));

    result_path = FLAGS_dest_dir + "/" + image_name + ".jpg";
    ACHECK(CameraVisualization(&camera_frame, result_path));
  }

  return true;
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo

int main(int argc, char* argv[]) {
  FLAGS_alsologtostderr = 1;
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::SetUsageMessage(
      "command line brew\n"
      "Usage: camera_benchmark <args>\n");
  apollo::perception::camera::TestDetection();
  return 0;
}
