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

#include "absl/strings/str_cat.h"
#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/base/distortion_model.h"
#include "modules/perception/camera/common/camera_frame.h"
#include "modules/perception/camera/lib/calibration_service/online_calibration_service/online_calibration_service.h"
#include "modules/perception/camera/lib/interface/base_calibration_service.h"
#include "modules/perception/camera/lib/lane/detector/denseline/denseline_lane_detector.h"
#include "modules/perception/camera/lib/lane/postprocessor/denseline/denseline_lane_postprocessor.h"
#include "modules/perception/camera/tools/lane_detection/lane_common.h"
#include "modules/perception/common/io/io_util.h"
#include "modules/perception/lib/utils/timer.h"

namespace apollo {
namespace perception {
namespace camera {

int lane_postprocessor_eval() {
  //  initialize lane detector
  LaneDetectorInitOptions init_options;
  LaneDetectorOptions detetor_options;
  init_options.conf_file = "config.pt";
  init_options.root_dir = "data/";
  base::BrownCameraDistortionModel model;
  if (!common::LoadBrownCameraIntrinsic("params/front_6mm_intrinsics.yaml",
                                        &model)) {
    AERROR << "LoadBrownCameraIntrinsic Error!";
    return -1;
  }
  init_options.base_camera_model = model.get_camera_model();

  std::shared_ptr<DenselineLaneDetector> detector(new DenselineLaneDetector);
  AINFO << "Detector: " << detector->Name();
  detector->Init(init_options);
  // Initialize lane postprocessor
  std::shared_ptr<DenselineLanePostprocessor> lane_postprocessor;
  lane_postprocessor.reset(new DenselineLanePostprocessor);
  LanePostprocessorInitOptions postprocessor_init_options;
  postprocessor_init_options.detect_config_root = "./data/";
  postprocessor_init_options.detect_config_name = "config.pt";
  postprocessor_init_options.root_dir = "./data/";
  postprocessor_init_options.conf_file = "lane_postprocessor_config.pt";
  lane_postprocessor->Init(postprocessor_init_options);
  LanePostprocessorOptions postprocessor_options;
  cyber::common::EnsureDirectory(FLAGS_save_dir);

  // Read image list
  std::ifstream list_file(FLAGS_list.c_str());
  std::string imname;
  std::vector<std::string> imnames;
  while (list_file >> imname) {
    imnames.push_back(imname);
  }
  // Read debug image list
  std::vector<std::string> debug_img_list;
  if (FLAGS_file_debug_list.length() > 2) {
    std::ifstream debug_list_file(FLAGS_file_debug_list.c_str());
    while (debug_list_file >> imname) {
      debug_img_list.push_back(imname);
    }
  }

  // Lane process for each image
  for (int i = 0; i < static_cast<int>(imnames.size()); ++i) {
    std::string impath = imnames[i];
    int pos1 = static_cast<int>(impath.rfind("/"));
    int pos2 = static_cast<int>(impath.rfind(".jpg"));
    FLAGS_file_title = impath.substr(pos1 + 1, pos2 - pos1 - 1);
    if (FLAGS_debug_file.length() > 0) {
      if (FLAGS_file_title.find(FLAGS_debug_file) == std::string::npos) {
        continue;
      }
    }
    // If debug list exists, check whether it in the list
    int debug_img_list_len = static_cast<int>(debug_img_list.size());
    if (debug_img_list_len > 0) {
      bool img_in_list_flag = false;
      for (int j = 0; j < debug_img_list_len; j++) {
        if (FLAGS_file_title == debug_img_list[j]) {
          img_in_list_flag = true;
          break;
        }
      }
      if (!img_in_list_flag) {
        continue;
      }
    }
    AINFO << "Process file: " << FLAGS_file_title;

    // Image data initialized
    CameraFrame frame;
    cv::Mat img = cv::imread(impath);
    CHECK(!img.empty()) << "input image is empty.";
    std::shared_ptr<base::SyncedMemory> img_gpu_data;
    int size = img.cols * img.rows * img.channels();
    img_gpu_data.reset(new base::SyncedMemory(size, true));
    memcpy(img_gpu_data->mutable_cpu_data(), img.data, size * sizeof(uint8_t));

    DataProvider data_provider;
    frame.data_provider = &data_provider;
    DataProvider::InitOptions dp_init_options;
    dp_init_options.image_height = img.rows;
    dp_init_options.image_width = img.cols;
    dp_init_options.device_id = 0;
    frame.data_provider->Init(dp_init_options);
    frame.data_provider->FillImageData(
        img.rows, img.cols, (const uint8_t*)(img_gpu_data->mutable_gpu_data()),
        "bgr8");

    // Set pitch angle
    float pitch_angle = 0.0f;
    // Set camera_ground_height (unit:meter)
    float camera_ground_height = 1.6f;
    // frame.pitch_angle = pitch_angle;
    // frame.camera_ground_height = camera_ground_height;
    frame.camera_k_matrix = model.get_intrinsic_params();
    CalibrationServiceInitOptions calibration_service_init_options;
    calibration_service_init_options.calibrator_working_sensor_name =
        "onsmi_obstacle";
    std::map<std::string, Eigen::Matrix3f> name_intrinsic_map;
    name_intrinsic_map["onsmi_obstacle"] = frame.camera_k_matrix;
    calibration_service_init_options.name_intrinsic_map = name_intrinsic_map;
    calibration_service_init_options.calibrator_method = "LaneLineCalibrator";
    calibration_service_init_options.image_height =
        static_cast<int>(model.get_height());
    calibration_service_init_options.image_width =
        static_cast<int>(model.get_width());
    std::shared_ptr<BaseCalibrationService> calibration_service;
    calibration_service.reset(
        BaseCalibrationServiceRegisterer::GetInstanceByName(
            "OnlineCalibration"));
    CHECK(calibration_service->Init(calibration_service_init_options));

    std::map<std::string, float> name_camera_ground_height_map;
    std::map<std::string, float> name_camera_pitch_angle_diff_map;
    name_camera_ground_height_map["onsmi_obstacle"] = camera_ground_height;
    name_camera_pitch_angle_diff_map["onsmi_obstacle"] = 0;
    calibration_service->SetCameraHeightAndPitch(
        name_camera_ground_height_map, name_camera_pitch_angle_diff_map,
        pitch_angle);
    frame.calibration_service = calibration_service.get();
    // Detect the lane image
    lib::Timer timer;
    timer.Start();
    detector->Detect(detetor_options, &frame);
    AINFO << "Detector finished!";
    timer.End("LaneDetector");

    timer.Start();
    // Postprocess
    lane_postprocessor->Process2D(postprocessor_options, &frame);
    lane_postprocessor->Process3D(postprocessor_options, &frame);
    timer.End("LanePostprocessor");
    std::string save_img_path;

    std::vector<unsigned char> lane_map;
    int lane_map_width = 0;
    int lane_map_height = 0;
    std::vector<ConnectedComponent> lane_ccs;
    std::vector<ConnectedComponent> select_lane_ccs;
    lane_postprocessor->GetLaneCCs(&lane_map, &lane_map_width, &lane_map_height,
                                   &lane_ccs, &select_lane_ccs);

    const std::vector<std::vector<LanePointInfo> >& detect_laneline_point_set =
        lane_postprocessor->GetLanelinePointSet();
    if (FLAGS_lane_line_debug) {
      save_img_path = absl::StrCat(FLAGS_save_dir, "/", FLAGS_file_title, "_0_",
                                   FLAGS_file_ext_name, ".jpg");
      const std::vector<LanePointInfo>& infer_point_set =
          lane_postprocessor->GetAllInferLinePointSet();
      show_all_infer_point_set(img, infer_point_set, save_img_path);

      save_img_path = absl::StrCat(FLAGS_save_dir, "/", FLAGS_file_title, "_1_",
                                   FLAGS_file_ext_name, ".jpg");
      show_detect_point_set(img, detect_laneline_point_set, save_img_path);
      AINFO << "detect_laneline_point_set num: "
            << detect_laneline_point_set.size();
    }
    // Draw the lane map, draw the connected_components
    if (FLAGS_lane_cc_debug) {
      save_img_path = absl::StrCat(FLAGS_save_dir, "/", FLAGS_file_title, "_2_",
                                   FLAGS_file_ext_name, ".jpg");
      show_lane_ccs(lane_map, lane_map_width, lane_map_height, lane_ccs,
                    select_lane_ccs, save_img_path);
    }
    if (FLAGS_lane_line_debug) {
      save_img_path = absl::StrCat(FLAGS_save_dir, "/", FLAGS_file_title, "_5_",
                                   FLAGS_file_ext_name, ".jpg");
      show_lane_lines(img, frame.lane_objects, save_img_path);
    }
    if (FLAGS_lane_result_output) {
      std::string save_path =
          absl::StrCat(FLAGS_save_dir, "/", FLAGS_file_title, ".txt");
      output_laneline_to_json(frame.lane_objects, save_path);
    }
  }

  return 0;
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  return apollo::perception::camera::lane_postprocessor_eval();
}
