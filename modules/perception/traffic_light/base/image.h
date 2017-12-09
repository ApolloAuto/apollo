/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef MODULES_PERCEPTION_TRAFFIC_LIGHT_BASE_IMAGE_H_
#define MODULES_PERCEPTION_TRAFFIC_LIGHT_BASE_IMAGE_H_

#include <map>
#include <memory>
#include <string>

#include "gflags/gflags.h"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/Image.h"

namespace apollo {
namespace perception {
namespace traffic_light {

// Camera id
enum CameraId {
  UNKNOWN = -1,
  LONG_FOCUS = 0,   // 25mm
  SHORT_FOCUS = 1,  // 6mm
  CAMERA_ID_COUNT = 2
};

const std::map<int, std::string> kCameraIdToStr = {
    {static_cast<int>(LONG_FOCUS), "long_focus_camera_25mm"},
    {static_cast<int>(SHORT_FOCUS), "short_focus_camera_6mm"}};

// @brief Image loaded from camera.
//       Warning: Image is not Thread Safe.
class Image {
 public:
  // @brief constructor
  Image() = default;

  // @brief init
  // @param [in] ts image's timestamp
  // @param [in] camera id
  // @param [in] image's data
  bool Init(const double &ts, const CameraId &device_id, const cv::Mat &mat);

  // @brief init
  // @param [in] ts image's timestamp
  // @param [in] camera id
  // @param [in] image's data
  bool Init(const double &ts, const CameraId &device_id,
            std::shared_ptr<const sensor_msgs::Image> image_data);
  // @brief return image's timestamp
  double ts() const;

  // @brief return image's device_id
  CameraId camera_id() const;
  // @brief return image's device_id_str
  std::string camera_id_str() const;

  // @brief return image's data
  cv::Mat mat() const;

  cv::Size size() const;
  // @brief cotain image.
  bool contain_mat() const { return contain_mat_; }
  bool contain_image() const { return contain_image_; }
  void set_ts(double ts) { timestamp_ = ts; }

  void set_device_id(CameraId camera_id) { camera_id_ = camera_id; }

  bool GenerateMat();

 private:
  bool contain_image_ = false;
  bool contain_mat_ = false;
  double timestamp_ = 0.0;                  // Image's timestamp
  CameraId camera_id_ = CameraId::UNKNOWN;  // camera's id
  cv::Mat mat_;                             // Image's data
  std::shared_ptr<const sensor_msgs::Image> image_data_;
  friend std::ostream &operator<<(std::ostream &os, const Image &image);
};

typedef std::shared_ptr<Image> ImageSharedPtr;

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_TRAFFIC_LIGHT_BASE_IMAGE_H_
