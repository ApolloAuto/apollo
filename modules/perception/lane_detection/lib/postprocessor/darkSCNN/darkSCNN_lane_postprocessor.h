/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
#pragma once

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "modules/perception/lane_detection/proto/darkSCNN.pb.h"
#include "modules/perception/lane_detection/proto/darkSCNN_postprocessor.pb.h"

#include "modules/common/util/eigen_defs.h"
#include "modules/perception/common/base/point.h"
#include "modules/perception/common/camera/common/camera_frame.h"
#include "modules/perception/common/lib/interface/base_calibration_service.h"
#include "modules/perception/common/lib/registerer/registerer.h"
#include "modules/perception/lane_detection/common/common_functions.h"
#include "modules/perception/lane_detection/interface/base_lane_postprocessor.h"

namespace apollo {
namespace perception {
namespace camera {

class DarkSCNNLanePostprocessor : public BaseLanePostprocessor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  template <class EigenType>
  using EigenVector = apollo::common::EigenVector<EigenType>;

 public:
  DarkSCNNLanePostprocessor() : BaseLanePostprocessor() {}

  virtual ~DarkSCNNLanePostprocessor() = default;

  bool Init(const LanePostprocessorInitOptions& options =
                LanePostprocessorInitOptions()) override;

  // @brief: detect lane from image.
  // @param [in]: options
  // @param [in/out]: frame
  // detected lanes should be filled, required,
  // 3D information of lane can be filled, optional.
  bool Process2D(const LanePostprocessorOptions& options,
                 CameraFrame* frame) override;
  // convert image point to the camera coordinate
  // & fit the line using polynomial
  bool Process3D(const LanePostprocessorOptions& options,
                 CameraFrame* frame) override;

  void SetIm2CarHomography(const Eigen::Matrix3d& homography_im2car) override {
    Eigen::Matrix3d im2car = homography_im2car;
    trans_mat_ = im2car.cast<float>();
    trans_mat_inv = trans_mat_.inverse();
  }

  // Todo(daohu527): Need define!
  std::vector<std::vector<LanePointInfo>> GetLanelinePointSet();
  std::vector<LanePointInfo> GetAllInferLinePointSet();

  std::string Name() const override;

 private:
  void ConvertImagePoint2Camera(CameraFrame* frame);
  // @brief: fit camera lane line using polynomial
  void PolyFitCameraLaneline(CameraFrame* frame);

 private:
  int input_offset_x_ = 0;
  int input_offset_y_ = 312;
  int lane_map_width_ = 640;
  int lane_map_height_ = 480;

  // this is actually the search range at the original image resolution
  int roi_height_ = 768;
  int roi_start_ = 312;
  int roi_width_ = 1920;

  // minimum number to fit a curve
  size_t minNumPoints_ = 8;

  int64_t time_1 = 0;
  int64_t time_2 = 0;
  int64_t time_3 = 0;
  int time_num = 0;

  float max_longitudinal_distance_ = 300.0f;
  float min_longitudinal_distance_ = 0.0f;

  // number of lane type (13)
  int lane_type_num_;

  lane::DarkSCNNLanePostprocessorParam lane_postprocessor_param_;

 private:
  Eigen::Matrix<float, 3, 3> trans_mat_;
  Eigen::Matrix<float, 3, 3> trans_mat_inv;
  // xy points for the ground plane, uv points for image plane
  EigenVector<EigenVector<Eigen::Matrix<float, 2, 1>>> xy_points;
  EigenVector<EigenVector<Eigen::Matrix<float, 2, 1>>> uv_points;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
