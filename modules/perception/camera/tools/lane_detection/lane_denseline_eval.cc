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
#include "modules/common/util/eigen_defs.h"
#include "modules/common/util/perf_util.h"
#include "modules/perception/base/distortion_model.h"
#include "modules/perception/camera/common/camera_frame.h"
#include "modules/perception/camera/lib/calibration_service/online_calibration_service/online_calibration_service.h"
#include "modules/perception/camera/lib/interface/base_calibration_service.h"
#include "modules/perception/camera/lib/lane/detector/darkSCNN/darkSCNN_lane_detector.h"
#include "modules/perception/camera/lib/lane/detector/denseline/denseline_lane_detector.h"
#include "modules/perception/camera/lib/lane/postprocessor/darkSCNN/darkSCNN_lane_postprocessor.h"
#include "modules/perception/camera/lib/lane/postprocessor/denseline/denseline_lane_postprocessor.h"
#include "modules/perception/camera/tools/common/util.h"
#include "modules/perception/camera/tools/lane_detection/lane_common.h"
#include "modules/perception/common/io/io_util.h"
#include "modules/perception/common/perception_gflags.h"

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

bool SaveLaneCCS(
    const std::shared_ptr<DarkSCNNLanePostprocessor>& lane_postprocessor) {
  int lane_map_width = 0;
  int lane_map_height = 0;
  std::vector<unsigned char> lane_map;
  std::vector<ConnectedComponent> lane_ccs;
  std::vector<ConnectedComponent> select_lane_ccs;
  // Todo(daohu527): need fix, denseline only
  // lane_postprocessor->GetLaneCCs(&lane_map, &lane_map_width,
  //                                &lane_map_height,
  //                                &lane_ccs, &select_lane_ccs);
  std::string save_img_path = absl::StrCat(FLAGS_save_dir, "/",
                                           FLAGS_file_title, "_2_",
                                           FLAGS_file_ext_name, ".jpg");
  show_lane_ccs(lane_map, lane_map_width, lane_map_height, lane_ccs,
                select_lane_ccs, save_img_path);
  return true;
}

bool SaveDetectPointSet(
    const std::shared_ptr<DarkSCNNLanePostprocessor>& lane_postprocessor,
    const cv::Mat& img) {
  std::string save_img_path = absl::StrCat(FLAGS_save_dir, "/",
                                           FLAGS_file_title, "_0_",
                                           FLAGS_file_ext_name, ".jpg");
  // Todo(daohu527): need fix, denseline only
  const std::vector<LanePointInfo> infer_point_set;
  // const std::vector<LanePointInfo>& infer_point_set =
      // lane_postprocessor->GetAllInferLinePointSet();
  show_all_infer_point_set(img, infer_point_set, save_img_path);

  save_img_path = absl::StrCat(FLAGS_save_dir, "/", FLAGS_file_title, "_1_",
                                FLAGS_file_ext_name, ".jpg");
  // Todo(daohu527): need fix, denseline only
  const std::vector<std::vector<LanePointInfo>> detect_laneline_point_set;
  // const std::vector<std::vector<LanePointInfo>>& detect_laneline_point_set =
  //     lane_postprocessor->GetLanelinePointSet();
  show_detect_point_set(img, detect_laneline_point_set, save_img_path);
  AINFO << "detect_laneline_point_set num: "
        << detect_laneline_point_set.size();
  return true;
}

bool lane_postprocessor_eval() {
  // 1. Init
  // 1.1 Initialize lane detector
  LaneDetectorInitOptions init_options;
  init_options.conf_file = "config_darkSCNN.pt";
  init_options.root_dir = "modules/perception/production/data/perception/camera/models/lane_detector"; // NOLINT
  base::BrownCameraDistortionModel model;
  if (!common::LoadBrownCameraIntrinsic("modules/perception/data/params/front_6mm_intrinsics.yaml", // NOLINT
                                        &model)) {
    AERROR << "LoadBrownCameraIntrinsic Error!";
    return -1;
  }
  init_options.base_camera_model = model.get_camera_model();

  std::shared_ptr<DarkSCNNLaneDetector> detector(new DarkSCNNLaneDetector);
  AINFO << "Detector: " << detector->Name();
  detector->Init(init_options);

  // 1.2 Initialize lane postprocessor
  std::shared_ptr<DarkSCNNLanePostprocessor> lane_postprocessor;
  lane_postprocessor.reset(new DarkSCNNLanePostprocessor);
  LanePostprocessorInitOptions postprocessor_init_options;
  postprocessor_init_options.detect_config_root = init_options.root_dir;
  postprocessor_init_options.detect_config_name = init_options.conf_file;
  postprocessor_init_options.root_dir = "modules/perception/production/data/perception/camera/models/lane_postprocessor/darkSCNN"; // NOLINT
  postprocessor_init_options.conf_file = "config.pt";
  lane_postprocessor->Init(postprocessor_init_options);

  // Read image list
  std::ifstream list_file(FLAGS_list.c_str());
  std::string imname;
  std::vector<std::string> imnames;
  while (list_file >> imname) {
    imnames.push_back(imname);
  }

  // Lane process for each image
  for (const std::string& impath : imnames) {
    int pos1 = static_cast<int>(impath.rfind("/"));
    int pos2 = static_cast<int>(impath.rfind(".jpg"));
    FLAGS_file_title = impath.substr(pos1 + 1, pos2 - pos1 - 1);
    AINFO << "Process file: " << FLAGS_file_title;

    // Image data initialized
    CameraFrame frame;
    DataProvider data_provider;
    frame.data_provider = &data_provider;
    ACHECK(InitDataProvider(&frame));

    std::string image_path =
        "modules/perception/testdata/camera/app/lane_images" + FLAGS_file_title;
    ACHECK(FillImage(&frame, image_path));

    // Calibration service
    float pitch_angle = 0.0f;
    // unit:meter
    float camera_ground_height = 1.6f;
    frame.camera_k_matrix = model.get_intrinsic_params();
    CalibrationServiceInitOptions calibration_service_init_options;
    calibration_service_init_options.calibrator_working_sensor_name =
        "onsmi_obstacle";
    apollo::common::EigenMap<std::string, Eigen::Matrix3f> name_intrinsic_map;
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
    ACHECK(calibration_service->Init(calibration_service_init_options));

    std::map<std::string, float> name_camera_ground_height_map;
    std::map<std::string, float> name_camera_pitch_angle_diff_map;
    name_camera_ground_height_map["onsmi_obstacle"] = camera_ground_height;
    name_camera_pitch_angle_diff_map["onsmi_obstacle"] = 0;
    calibration_service->SetCameraHeightAndPitch(
        name_camera_ground_height_map, name_camera_pitch_angle_diff_map,
        pitch_angle);
    frame.calibration_service = calibration_service.get();

    // 2. Detect the lane image
    apollo::common::util::Timer timer;
    timer.Start();
    // 2.1 Detect
    LaneDetectorOptions detetor_options;
    detector->Detect(detetor_options, &frame);
    AINFO << "Detector finished!";
    timer.End("LaneDetector");

    timer.Start();
    // 2.2 Postprocess
    LanePostprocessorOptions postprocessor_options;
    lane_postprocessor->Process2D(postprocessor_options, &frame);
    lane_postprocessor->Process3D(postprocessor_options, &frame);
    timer.End("LanePostprocessor");

    // 3. Save result
    std::string save_img_path;
    cyber::common::EnsureDirectory(FLAGS_save_dir);
    cv::Mat img = cv::imread(impath);
    if (FLAGS_lane_line_debug) {
      SaveDetectPointSet(lane_postprocessor, img);
    }
    // Draw the lane map, draw the connected_components
    if (FLAGS_lane_cc_debug) {
      SaveLaneCCS(lane_postprocessor);
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
  apollo::perception::camera::lane_postprocessor_eval();
  return 0;
}
