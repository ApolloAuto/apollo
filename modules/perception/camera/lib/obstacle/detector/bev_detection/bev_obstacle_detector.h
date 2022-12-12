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

  bool IsEnabled() const override { return enable_;}

  std::string Name() const override { return name_;}

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
      -1.40307297e-03, 9.07780395e-06,  4.84838307e-01,  -5.43047376e-02,
      -1.40780103e-04, 1.25770375e-05,  1.04126692e+00,  7.67668605e-01,
      -1.02884378e-05, -1.41007011e-03, 1.02823459e-01,  -3.07415128e-01,
      0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00,
      -9.39000631e-04, -7.65239349e-07, 1.14073277e+00,  4.46270645e-01,
      1.04998052e-03,  1.91798881e-05,  2.06218868e-01,  7.42717385e-01,
      1.48074005e-05,  -1.40855671e-03, 7.45946690e-02,  -3.16081315e-01,
      0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00,
      -7.0699735e-04,  4.2389297e-07,   -5.5183989e-01,  -5.3276348e-01,
      -1.2281288e-03,  2.5626015e-05,   1.0212017e+00,   6.1102939e-01,
      -2.2421273e-05,  -1.4170362e-03,  9.3639769e-02,   -3.0863306e-01,
      0.0000000e+00,   0.0000000e+00,   0.0000000e+00,   1.0000000e+00,
      2.2227580e-03,   2.5312484e-06,   -9.7261822e-01,  9.0684637e-02,
      1.9360810e-04,   2.1347081e-05,   -1.0779887e+00,  -7.9227984e-01,
      4.3742721e-06,   -2.2310747e-03,  1.0842450e-01,   -2.9406491e-01,
      0.0000000e+00,   0.0000000e+00,   0.0000000e+00,   1.0000000e+00,
      5.97175560e-04,  -5.88774265e-06, -1.15893924e+00, -4.49921310e-01,
      -1.28312141e-03, 3.58297058e-07,  1.48300052e-01,  1.14334166e-01,
      -2.80917516e-06, -1.41527120e-03, 8.37693438e-02,  -2.36765608e-01,
      0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00,
      3.6048229e-04,   3.8333174e-06,   7.9871160e-01,   4.3321830e-01,
      1.3671946e-03,   6.7484652e-06,   -8.4722507e-01,  1.9411178e-01,
      7.5027779e-06,   -1.4139183e-03,  8.2083985e-02,   -2.4505949e-01,
      0.0000000e+00,   0.0000000e+00,   0.0000000e+00,   1.0000000e+00};
  Eigen::Matrix4d imu2lidar_matrix_rt_;

  std::vector<float> mean_{103.530, 116.280, 123.675};
  std::vector<float> std_{57.375, 57.120, 58.395};

  std::shared_ptr<paddle_infer::Predictor> predictor_;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
