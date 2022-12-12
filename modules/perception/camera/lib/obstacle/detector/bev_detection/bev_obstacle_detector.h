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
#include <numeric>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "paddle/include/paddle_inference_api.h"

#include "modules/perception/pipeline/proto/stage/bev_obstacle_detection_config.pb.h"

#include "modules/perception/camera/lib/interface/base_obstacle_detector.h"
#include "modules/perception/pipeline/stage.h"

namespace apollo {
namespace perception {
namespace camera {

class BEVObstacleDetector : public BaseObstacleDetector {
 public:
  BEVObstacleDetector() : BaseObstacleDetector(){};
  virtual ~BEVObstacleDetector() = default;

  bool Init(const StageConfig& stage_config) override;

  bool Init(const ObstacleDetectorInitOptions& options =
                ObstacleDetectorInitOptions()) override;

  bool Detect(const ObstacleDetectorOptions& options,
              CameraFrame* frame) override;

  bool Process(DataFrame* data_frame) override;

  void Resize(const cv::Mat& img, int resized_h, int resized_w,
              cv::Mat* resize_img);

  void Normalize(const std::vector<float>& mean, const std::vector<float>& std,
                 float scale, cv::Mat* im);

  void Mat2Vec(const cv::Mat& im, float* data);

  void FilterScore(const std::vector<float>& box3d,
                   const std::vector<int64_t>& label_preds,
                   const std::vector<float>& scores, float score_threshold,
                   std::vector<float>* box3d_filtered,
                   std::vector<int64_t>* label_preds_filtered);

  bool IsEnabled() const override { return enable_; }

  std::string Name() const override { return name_; }

 private:
  void Run(paddle_infer::Predictor* predictor,
           const std::vector<int>& images_shape,
           const std::vector<float>& images_data,
           const std::vector<int>& k_shape, const std::vector<float>& k_data,
           std::vector<float>* boxes, std::vector<float>* scores,
           std::vector<int64_t>* labels);

  void Lidar2cam(const Eigen::Matrix4f& imu2camera,
                 const Eigen::Matrix4f& imu2lidar,
                 Eigen::Matrix4f* lidar2camera);

  void GetMatrixRT(const Eigen::Quaterniond& rotation_quaternion,
                   const Eigen::Vector3f& translation,
                   Eigen::Matrix4f* matrix_rt);

  void GetImg2LidarMatrix(const Eigen::Matrix4f& imu2cam_matrix_rt,
                          const Eigen::Matrix3f& cam_intrinstic_matrix_3f,
                          const Eigen::Matrix4f& imu2lidar_matrix_rt,
                          Eigen::Matrix4f* img2lidar_matrix_rt);

  void GetImg2LidarMatrix(const Eigen::Matrix4f& lidar2cam_matrix_rt,
                          const Eigen::Matrix3f& cam_intrinstic_matrix_3f,
                          Eigen::Matrix4f* img2lidar_matrix_rt);

      private : int gpu_id_ = 0;
  int frame_array_size_ = 6;
  int num_output_box_feature_ = 7;

  int image_height_ = 900;
  int image_width_ = 1600;
  int image_height_resized_ = 450;
  int image_width_resized_ = 800;
  int img_height_crop_ = 320;
  int img_width_crop_ = 800;

  double preprocess_time_ = 0.0;
  double inference_time_ = 0.0;

  std::vector<int> images_shape_{1, 6, 3, img_height_crop_, img_width_crop_};
  std::vector<float> images_data_;
  std::vector<int> k_shape_{1, 6, 4, 4};
  std::vector<float> k_data_;

  std::vector<float> mean_{103.530, 116.280, 123.675};
  std::vector<float> std_{57.375, 57.120, 58.395};

  BEVObstacleDetectionConfig bev_obstacle_detection_config_;
  std::shared_ptr<paddle_infer::Predictor> predictor_;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo