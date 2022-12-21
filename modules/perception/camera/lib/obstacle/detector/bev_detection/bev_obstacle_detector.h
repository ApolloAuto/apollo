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
#include <numeric>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "paddle/include/paddle_inference_api.h"
#include "yaml-cpp/yaml.h"

#include "modules/perception/pipeline/proto/stage/bev_obstacle_detection_config.pb.h"

#include "modules/perception/camera/lib/interface/base_obstacle_detector.h"
#include "modules/perception/pipeline/stage.h"

namespace apollo {
namespace perception {
namespace camera {

class BEVObstacleDetector : public BaseObstacleDetector {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  BEVObstacleDetector() : BaseObstacleDetector() {}
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
                   std::vector<int64_t>* label_preds_filtered,
                   std::vector<float>* scores_filtered);

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

  bool LoadExtrinsics(const std::string& yaml_file,
                      Eigen::Matrix4d* camera_extrinsic);

  void GetObjects(const std::vector<float>& detections,
                  const std::vector<int64_t>& labels,
                  const std::vector<float>& scores,
                  camera::CameraFrame* camera_frame);

  void FillBBox3d(const float* bbox, const Eigen::Affine3d& world2cam_pose,
                  const Eigen::Matrix4d& imu2cam_matrix_rt,
                  const Eigen::Matrix4d& imu2lidar_matrix_rt,
                  base::ObjectPtr obj);

  base::ObjectSubType GetObjectSubType(const int label);

  bool Nuscenes2Apollo(const std::vector<float>& bbox_nuscenes,
                       std::vector<float>* bbox_apollo);

 private:
  int gpu_id_ = 0;
  int frame_array_size_ = 6;
  int num_output_box_feature_ = 9;

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
  std::vector<float> k_data_{
      0.00159625,  -0.00001496, -0.66888732, -0.0041606,  0.00001794,
      0.00008723,  0.9818714,   0.75139001,  -0.000014,   -0.00159395,
      0.22787872,  -0.31089077, 0.,          0.,          0.,
      1.,          0.00085403,  -0.00000108, 0.49469646,  0.49969287,
      -0.00134016, 0.00009974,  1.07404512,  0.60950301,  -0.00008474,
      -0.00158828, 0.22014857,  -0.29333971, 0.,          0.,
      0.,          1.,          0.00090349,  -0.00002144, -1.19441294,
      -0.49988666, 0.00130713,  0.00008375,  0.01826113,  0.60405286,
      0.00005675,  -0.00158765, 0.16909284,  -0.30713859, 0.,
      0.,          0.,          1.,          -0.00250957, -0.00002087,
      1.06997893,  -0.00947018, 0.00002077,  0.00009896,  -1.0188297,
      -0.91697425, 0.0000217,   -0.00250771, 0.22327503,  -0.31500559,
      0.,          0.,          0.,          1.,          -0.00050857,
      -0.00002523, -0.73401539, -0.48226721, 0.00150894,  0.00005944,
      -0.95155324, 0.07532162,  0.00006439,  -0.00159233, 0.14132135,
      -0.27632716, 0.,          0.,          0.,          1.,
      -0.00061203, 0.00002302,  1.17408651,  0.4680249,   -0.00147655,
      0.00006982,  0.21952066,  0.08716113,  -0.00007331, -0.00159836,
      0.18871528,  -0.28457765, 0.,          0.,          0.,
      1.};

  Eigen::Matrix4d imu2lidar_matrix_rt_;

  std::vector<float> mean_{103.530, 116.280, 123.675};
  std::vector<float> std_{57.375, 57.120, 58.395};

  std::shared_ptr<paddle_infer::Predictor> predictor_;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
