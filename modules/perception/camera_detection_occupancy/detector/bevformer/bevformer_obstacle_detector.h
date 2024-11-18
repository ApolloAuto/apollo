/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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

#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "Eigen/Core"
#include "opencv2/opencv.hpp"

#include "modules/perception/camera_detection_occupancy/detector/bevformer/proto/model_param.pb.h"

#include "modules/perception/camera_detection_occupancy/camera_frame.h"
#include "modules/perception/camera_detection_occupancy/interface/base_obstacle_detector.h"
#include "modules/perception/common/base/blob.h"
#include "modules/perception/common/base/object_types.h"
#include "modules/perception/common/onboard/transform_wrapper/transform_wrapper.h"
#include "modules/transform/buffer.h"

namespace apollo {
namespace perception {
namespace camera {

using apollo::transform::Buffer;
class BEVFORMERObstacleDetector : public BaseObstacleDetector {
 public:
  BEVFORMERObstacleDetector() = default;
  virtual ~BEVFORMERObstacleDetector() = default;
  /**
   * @brief Init ObstacleDetector constructor.
   *
   * @param options the option object of obstacle
   * @return true
   * @return false
   */
  bool Init(const ObstacleDetectorInitOptions &options =
                ObstacleDetectorInitOptions()) override;
  /**
   * @brief Get obstacles detection result
   *
   * @param frame  camera frame
   * @return true
   * @return false
   */
  bool Detect(CameraFrame *frame) override;

  std::string Name() const override { return "BEVObstacleDetector"; }

 private:
  bool InitTypes(const bevformer::ModelParam &model_param);

  void InitImageSize(const bevformer::ModelParam &model_param);

  bool ImagePreprocess(const CameraFrame *frame,
                       base::BlobPtr<float> input_img_blob);
  void FillCanBus(const CameraFrame *frame, base::BlobPtr<float> can_bus_blob);
  bool ImagePreprocessGPU(const CameraFrame *frame,
                          base::BlobPtr<float> input_img_blob);
  bool ImageExtrinsicPreprocess(const CameraFrame *frame,
                                base::BlobPtr<float> input_img2lidar_blob);

  bool Nuscenes2Apollo(std::vector<base::ObjectPtr> *objects);

  void Mat2Vec(const cv::Mat &im, float *data);

  void GetObjects(const CameraFrame *frame, const float *outputs_scores,
                  const float *outputs_coords,
                  std::vector<base::ObjectPtr> *objects);

  bool GetOccResults(CameraFrame *frame, const float *outputs_occupancy);

 private:
  int height_;
  int width_;

  int gpu_id_ = 0;
  int max_batch_size_ = 32;
  cudaStream_t stream_ = nullptr;
  // for nuscenes six camera
  std::vector<float> k_nuscenes_lidar2img_{
      1.26315285e+03,  8.20961550e+02,  2.39035795e+01,  -6.07673494e+02,
      6.61316935e+00,  5.15120214e+02,  -1.25698017e+03, -7.86354408e+02,
      -3.95160255e-03, 9.99817517e-01,  1.86900441e-02,  -7.62910510e-01,
      0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00,
      1.36956207e+03,  -6.04983867e+02, -2.90382932e+01, -3.15916033e+02,
      3.99862161e+02,  3.04064067e+02,  -1.25807304e+03, -7.97115537e+02,
      8.33862795e-01,  5.51951686e-01,  4.70912802e-03,  -7.53284567e-01,
      0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00,
      5.10102547e+01,  1.51619727e+03,  3.66401186e+01,  -8.63314902e+02,
      -3.89866270e+02, 3.05282552e+02,  -1.26667835e+03, -7.77329408e+02,
      -8.19828451e-01, 5.72492498e-01,  1.15607703e-02,  -7.39298163e-01,
      0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00,
      -8.12919287e+02, -8.25472117e+02, -1.42047254e+01, -7.56204850e+02,
      5.79180367e+00,  -4.75682694e+02, -8.12798555e+02, -6.61898527e+02,
      -4.51723964e-03, -9.99961261e-01, -7.55454093e-03, -9.09695896e-01,
      0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00,
      -1.14954471e+03, 9.40916235e+02,  8.06648094e+00,  -6.46751669e+02,
      -4.42244125e+02, -1.14568251e+02, -1.27024460e+03, -5.19557827e+02,
      -9.48195535e-01, -3.16334599e-01, -2.92856511e-02, -4.34020102e-01,
      0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00,
      3.04831143e+02,  -1.46336936e+03, -6.04997023e+01, -4.76695738e+01,
      4.61395173e+02,  -1.27936752e+02, -1.26819508e+03, -5.58714538e+02,
      9.34193888e-01,  -3.56238546e-01, -1.93875899e-02, -4.27659445e-01,
      0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00};

  bevformer::ModelParam model_param_;
  ObstacleDetectorInitOptions options_;
  std::vector<base::ObjectSubType> types_;

  int num_decoder_layer_;
  int num_queries_;
  int num_classes_;
  int num_classes_det_;
  int num_classes_occ_;
  int code_size_;
  int class_blob_start_index_;
  int class_blob_end_index_;
  int box_blob_start_index_;
  float *use_prev_bev_;
  int prev_bev_size_ = 1;
  std::shared_ptr<onboard::TransformWrapper> trans_wrapper_;
  Buffer *tf2_buffer_ = Buffer::Instance();

  std::vector<float> mean_bgr_{0, 0, 0};
  std::vector<float> std_bgr_{0, 0, 0};
  bool resize_flag_ = false;
  std::vector<float> resized_lidar2img_;
  std::vector<double> last_can_bus_;

  int occ_blob_index_ = 3;
  int num_voxels_;
  float occ_threshold_ = 0.25;
  float occ_xmin_ = -20.0;
  float occ_xmax_ = 20.0;
  float occ_ymin_ = -20.0;
  float occ_ymax_ = 20.0;
  float occ_zmin_ = -20.0;
  float occ_zmax_ = 20.0;
  float voxel_size_ = 0.2;
  int occ_x_grid_;
  int occ_y_grid_;
  int occ_z_grid_;
  std::vector<float> no_pad_image_shape_ = {450.0, 800.0};
  std::vector<std::pair<int, int>> occ_results_;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
