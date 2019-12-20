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
#include <fstream>
#include "gtest/gtest.h"

#include "cyber/common/log.h"
#include "modules/perception/camera/lib/calibration_service/online_calibration_service/online_calibration_service.h"  // NOLINT
#include "modules/perception/camera/lib/obstacle/postprocessor/location_refiner/location_refiner_obstacle_postprocessor.h"  // NOLINT
#include "modules/perception/common/io/io_util.h"

namespace apollo {
namespace perception {
namespace camera {

static bool load_camera_k_mat_and_ground_pitch_height(
    const std::string &fname, float k_mat[9], int *image_width,
    int *image_height, float *camera_pitch, float *camera_height) {
  if (image_width == nullptr || image_height == nullptr ||
      camera_pitch == nullptr || camera_height == nullptr) {
    AERROR << "Null pointer input for loading this file:  " << fname;
    return false;
  }
  std::fstream fin(fname);
  if (!fin.is_open()) {
    AERROR << "Fail to load the camera k matrix: " << fname;
    return false;
  }
  float wh_flt[2] = {0};
  fin >> wh_flt[0] >> wh_flt[1];
  *image_width = common::IRound(wh_flt[0]);
  *image_height = common::IRound(wh_flt[1]);
  fin >> k_mat[0] >> k_mat[1] >> k_mat[2] >> k_mat[3] >> k_mat[4] >> k_mat[5] >>
      k_mat[6] >> k_mat[7] >> k_mat[8];
  fin >> camera_pitch[0] >> camera_height[0];
  fin.close();
  AINFO << "cx: " << k_mat[2];
  AINFO << "cy: " << k_mat[5];
  AINFO << "pitch: " << camera_pitch[0];
  AINFO << "height: " << camera_height[0];
  return true;
}

struct ObjectInfo {
  std::string type = "";
  float truncated = 0.0f;
  int occluded = 0;
  float alpha = 0.0f;
  float bbox[4] = {0};
  float size_hwl[3] = {0};
  float location[3] = {0};
  float rotation_y = 0.0f;
  float score = 1.0f;
};

static bool load_label_from_file(const std::string &filename,
                                 std::vector<ObjectInfo> *labels) {
  labels->clear();
  std::fstream fin(filename);
  if (!fin.is_open()) {
    AERROR << "Fail to open the lab file: " << filename << std::endl;
    return false;
  }
  ObjectInfo label_in;
  while (fin >> label_in.type >> label_in.truncated >> label_in.occluded >>
         label_in.alpha >> label_in.bbox[0] >> label_in.bbox[1] >>
         label_in.bbox[2] >> label_in.bbox[3] >> label_in.size_hwl[0] >>
         label_in.size_hwl[1] >> label_in.size_hwl[2] >> label_in.location[0] >>
         label_in.location[1] >> label_in.location[2] >> label_in.rotation_y >>
         label_in.score) {
    labels->push_back(label_in);
  }
  fin.close();
  return true;
}

TEST(LocationRefinedObstaclePostProcessorTestOnlineCalibration,
     location_refiner_obstacle_postprocessor_test_onlinecalibration) {
  std::string fname_k_mat =
      "/apollo/modules/perception/testdata/"
      "camera/lib/obstacle/postprocessor/location_refiner/"
      "params/k_mat.txt";
  std::string fname_detection =
      "/apollo/modules/perception/testdata/"
      "camera/lib/obstacle/postprocessor/location_refiner/"
      "data/detection.txt";
  float k_mat[9] = {0};
  std::vector<ObjectInfo> detections = {};
  int width = 0;
  int height = 0;
  float camera_pitch = 0.0f;
  float camera_height = 0.0f;
  load_camera_k_mat_and_ground_pitch_height(fname_k_mat, k_mat, &width, &height,
                                            &camera_pitch, &camera_height);
  load_label_from_file(fname_detection, &detections);

  CameraFrame frame;
  DataProvider data_provider;
  frame.data_provider = &data_provider;
  if (frame.track_feature_blob == nullptr) {
    frame.track_feature_blob.reset(new base::Blob<float>());
  }
  DataProvider::InitOptions dp_init_options;
  dp_init_options.sensor_name = "front_12mm";
  dp_init_options.image_height = height;
  dp_init_options.image_width = width;
  dp_init_options.device_id = 0;
  frame.data_provider->Init(dp_init_options);
  for (size_t i = 0; i < 3; i++) {
    size_t i3 = i * 3;
    for (size_t j = 0; j < 3; j++) {
      frame.camera_k_matrix(i, j) = k_mat[i3 + j];
    }
  }

  ObstaclePostprocessorOptions postprocessor_options;
  ObstaclePostprocessorInitOptions postprocessor_init_options;

  // postprocessor
  BaseObstaclePostprocessor *postprocessor =
      BaseObstaclePostprocessorRegisterer::GetInstanceByName(
          "LocationRefinerObstaclePostprocessor");
  EXPECT_FALSE(postprocessor->Init(postprocessor_init_options));
  postprocessor_init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/lib/obstacle/postprocessor/location_refiner/";
  postprocessor_init_options.conf_file = "config.pt";
  EXPECT_TRUE(postprocessor->Init(postprocessor_init_options));
  EXPECT_TRUE(postprocessor->Process(postprocessor_options, &frame));
  //  frame.detected_objects.push_back(nullptr);
  //  EXPECT_FALSE(postprocessor->Process(postprocessor_options, &frame));
  //  frame.detected_objects.clear();

  std::vector<std::string> candidate_type_list = {"car", "van", "truck", "bus"};
  for (auto &obj : detections) {
    if (std::find(candidate_type_list.begin(), candidate_type_list.end(),
                  obj.type) == candidate_type_list.end()) {
      continue;
    } else if (obj.size_hwl[0] < 1.0f || obj.bbox[2] < obj.bbox[0] + 1.0f ||
               obj.bbox[3] < obj.bbox[1] + 1.0f) {
      continue;
    }

    base::ObjectPtr detected_obj(new base::Object);
    detected_obj->camera_supplement.box.xmin = obj.bbox[0];
    detected_obj->camera_supplement.box.ymin = obj.bbox[1];
    detected_obj->camera_supplement.box.xmax = obj.bbox[2];
    detected_obj->camera_supplement.box.ymax = obj.bbox[3];

    detected_obj->size(2) = obj.size_hwl[0];
    detected_obj->size(1) = obj.size_hwl[1];
    detected_obj->size(0) = obj.size_hwl[2];

    detected_obj->camera_supplement.local_center(0) = obj.location[0];
    detected_obj->camera_supplement.local_center(1) = obj.location[1];
    detected_obj->camera_supplement.local_center(2) = obj.location[2];
    detected_obj->camera_supplement.local_center(1) -= obj.size_hwl[0] / 2;

    float box_cent_x = (obj.bbox[0] + obj.bbox[2]) / 2;
    Eigen::Vector3f image_point_low_center(box_cent_x, obj.bbox[3], 1);
    Eigen::Vector3f point_in_camera =
        static_cast<Eigen::Matrix<float, 3, 1, 0, 3, 1>>(
            frame.camera_k_matrix.inverse() * image_point_low_center);
    float theta_ray =
        static_cast<float>(atan2(point_in_camera.x(), point_in_camera.z()));
    detected_obj->camera_supplement.alpha = theta_ray - obj.rotation_y;
    frame.detected_objects.push_back(detected_obj);
  }

  AINFO << camera_pitch << ", " << camera_height;
  OnlineCalibrationService online_calib_service;
  CalibrationServiceInitOptions options;
  Eigen::Matrix3f intrinsic;
  intrinsic << k_mat[0], k_mat[1], k_mat[2], k_mat[3], k_mat[4], k_mat[5],
      k_mat[6], k_mat[7], k_mat[8];
  std::map<std::string, Eigen::Matrix3f> name_intrinsic_map;
  name_intrinsic_map.insert(
      std::pair<std::string, Eigen::Matrix3f>("front_12mm", intrinsic));
  CalibrationServiceInitOptions calibration_service_init_options;
  calibration_service_init_options.calibrator_working_sensor_name =
      "front_12mm";
  calibration_service_init_options.name_intrinsic_map = name_intrinsic_map;
  calibration_service_init_options.calibrator_method = "LaneLineCalibrator";
  calibration_service_init_options.image_height = 1080;
  calibration_service_init_options.image_width = 1920;
  online_calib_service.Init(calibration_service_init_options);
  std::map<std::string, float> name_camera_ground_height_map;
  std::map<std::string, float> name_camera_pitch_angle_diff_map;
  name_camera_ground_height_map["front_12mm"] = camera_height;
  name_camera_pitch_angle_diff_map["front_12mm"] = 0;
  online_calib_service.SetCameraHeightAndPitch(name_camera_ground_height_map,
                                               name_camera_pitch_angle_diff_map,
                                               camera_pitch);
  frame.calibration_service =
      dynamic_cast<BaseCalibrationService *>(&online_calib_service);
  EXPECT_TRUE(postprocessor->Process(postprocessor_options, &frame));

  frame.calibration_service->BuildIndex();
  AINFO << "number of objects: " << frame.detected_objects.size();
  EXPECT_TRUE(postprocessor->Process(postprocessor_options, &frame));

  // corner cases
  postprocessor_options.do_refinement_with_calibration_service = false;
  EXPECT_TRUE(postprocessor->Process(postprocessor_options, &frame));
  postprocessor_options.do_refinement_with_calibration_service = true;
  frame.calibration_service = nullptr;
  EXPECT_TRUE(postprocessor->Process(postprocessor_options, &frame));
  postprocessor_options.do_refinement_with_calibration_service = false;
  EXPECT_TRUE(postprocessor->Process(postprocessor_options, &frame));

  delete postprocessor;
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
