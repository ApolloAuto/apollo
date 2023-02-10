/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
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

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "opencv2/opencv.hpp"
#include "paddle/include/paddle_inference_api.h"

#include "modules/perception/pipeline/proto/stage/caddn_detection_config.pb.h"

#include "cyber/common/file.h"
#include "modules/perception/base/box.h"
#include "modules/perception/base/object_types.h"
#include "modules/perception/camera/common/util.h"
#include "modules/perception/camera/lib/interface/base_feature_extractor.h"
#include "modules/perception/camera/lib/interface/base_obstacle_detector.h"
#include "modules/perception/inference/inference.h"
#include "modules/perception/inference/utils/resize.h"
#include "modules/perception/inference/utils/util.h"
#include "modules/perception/pipeline/stage.h"

namespace apollo {
namespace perception {
namespace camera {

class CaddnObstacleDetector : public BaseObstacleDetector {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  CaddnObstacleDetector() : BaseObstacleDetector() {}
  virtual ~CaddnObstacleDetector() = default;
  bool Init(const StageConfig &stage_config) override;

  bool Init(const ObstacleDetectorInitOptions &options =
                ObstacleDetectorInitOptions()) override;

  bool Detect(const ObstacleDetectorOptions &options,
              CameraFrame *frame) override;

  bool Process(DataFrame *data_frame) override;

  bool IsEnabled() const override { return enable_; }

  std::string Name() const override { return "CaddnObstacleDetector"; }

 private:
  void Run(paddle_infer::Predictor *predictor,
           const std::vector<int> &images_shape,
           const std::vector<float> &images_data,
           const std::vector<int> &cam_shape,
           const std::vector<float> &cam_data,
           const std::vector<int> &lidar_shape,
           const std::vector<float> &lidar_data, std::vector<float> *boxes,
           std::vector<float> *labels, std::vector<float> *scores);

  void GetCaddnObjects(std::vector<base::ObjectPtr> *objects, int width,
                       int height, const std::vector<float> *boxes,
                       const std::vector<float> *labels,
                       const std::vector<float> *scores);

  void Bbox3dLidar2Camera(const Eigen::Matrix<float, 3, 4> &V2C,
                          const Eigen::Matrix<float, 3, 3> &R,
                          const float *bbox_lidar,
                          std::vector<float> *bbox_camera);

  void Bbox3d2Bbox2d(const Eigen::Matrix<float, 3, 3> &K,
                     const float *bbox3d_camera, std::vector<float> *bbox_2d);

  void FillCaddnBase(base::ObjectPtr obj, const float *bbox, int width,
                     int height);

  void FillCaddnBbox3d(base::ObjectPtr obj, const float *bbox);

  void RecoverCaddnBbox(int roi_w, int roi_h, int offset_y,
                        std::vector<base::ObjectPtr> *objects);

  base::ObjectSubType GetCaddnObjectSubtype(int cls);

  void Mat2Vec(const cv::Mat *im, float *data);

  void Normalize(cv::Mat *im, const std::vector<float> &mean,
                 const std::vector<float> &std, float scale);

  int image_height_ = 1080;
  int image_width_ = 1920;

  int height_ = 640;
  int width_ = 960;
  int offset_y_ = 0;
  float scale_ = 1.0f / 255.0f;

  std::vector<float> mean_val_{0.485, 0.456, 0.406};
  std::vector<float> std_val_{0.229, 0.224, 0.225};

  std::vector<int> input_cam_shape_ = {1, 3, 4};
  std::vector<float> input_lidar_data_ = {
      0.0048523,   -0.9999298, -0.01081266, -0.00711321,
      -0.00302069, 0.01079808, -0.99993706, -0.06176636,
      0.99998367,  0.00488465, -0.00296808, -0.26739058,
      0.,          0.,         0.,          1.};
  std::vector<int> input_lidar_shape_ = {1, 4, 4};

  Eigen::Matrix3f camera_k_matrix_ = Eigen::Matrix3f::Identity();

  std::shared_ptr<paddle_infer::Predictor> predictor_;
  paddle_infer::Config config_;
  int gpu_id_ = 0;
};
}  // namespace camera
}  // namespace perception
}  // namespace apollo
