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

#include <string>
#include <vector>

#include "Eigen/Core"
#include "opencv2/opencv.hpp"

#include "modules/perception/camera_detection_bev/detector/petr/proto/model_param.pb.h"

#include "modules/perception/camera_detection_bev/interface/base_obstacle_detector.h"
#include "modules/perception/common/base/blob.h"
#include "modules/perception/common/base/object_types.h"

namespace apollo {
namespace perception {
namespace camera {

class BEVObstacleDetector : public BaseObstacleDetector {
 public:
  BEVObstacleDetector() = default;
  virtual ~BEVObstacleDetector() = default;
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
  void InitImageSize(const petr::ModelParam &model_param);

  bool ImagePreprocess(const CameraFrame *frame,
                       base::BlobPtr<float> input_img_blob);

  bool ImageExtrinsicPreprocess(base::BlobPtr<float> input_img2lidar_blob);

  bool Nuscenes2Apollo(std::vector<base::ObjectPtr> *objects);

  void Mat2Vec(const cv::Mat &im, float *data);

 private:
  int height_;
  int width_;

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

  petr::ModelParam model_param_;
  ObstacleDetectorInitOptions options_;
  std::vector<base::ObjectSubType> types_;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
