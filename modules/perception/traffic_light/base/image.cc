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

#include "modules/perception/traffic_light/base/image.h"

#include "cv_bridge/cv_bridge.h"

#include "modules/common/log.h"
#include "modules/perception/traffic_light/util/color_space.h"

DEFINE_int32(double_show_precision, 14,
             "When output a double data, the precision.");

namespace apollo {
namespace perception {
namespace traffic_light {

bool Image::Init(const double &ts, const CameraId &device_id,
                 const cv::Mat &mat) {
  contain_mat_ = true;
  contain_image_ = true;
  timestamp_ = ts, camera_id_ = device_id, mat_ = mat.clone();
  ADEBUG << *this << " init.";
  return true;
}
bool Image::Init(const double &ts, const CameraId &device_id,
                 boost::shared_ptr<const sensor_msgs::Image> image_data) {
  contain_mat_ = false;
  contain_image_ = true;
  timestamp_ = ts, camera_id_ = device_id, image_data_ = image_data;
  ADEBUG << *this << " init.";
  return true;
}

double Image::ts() const { return timestamp_; }

CameraId Image::camera_id() const { return camera_id_; }

std::string Image::camera_id_str() const {
  if (kCameraIdToStr.find(camera_id_) == kCameraIdToStr.end()) {
    return "unkown camera";
  }
  return kCameraIdToStr.at(camera_id_);
}
bool Image::GenerateMat() {
  if (!contain_mat_) {
    try {
      if (image_data_->encoding.compare("yuyv") == 0) {
        unsigned char *yuv = (unsigned char *)&(image_data_->data[0]);
        mat_ = cv::Mat(image_data_->height, image_data_->width, CV_8UC3);
        Yuyv2rgb(yuv, mat_.data, image_data_->height * image_data_->width);
        cv::cvtColor(mat_, mat_, CV_RGB2BGR);
      }

      contain_mat_ = true;
      AINFO << "Generate done " << mat_.size();
    } catch (const cv_bridge::Exception &e) {
      AERROR << "TLPreprocessorSubnode trans msg to cv::Mat failed."
             << e.what();
      return false;
    }
  }
  return true;
}
cv::Mat Image::mat() const { return mat_; }
cv::Size Image::size() const {
  if (contain_mat_) {
    return mat_.size();
  } else {
    return cv::Size(image_data_->width, image_data_->height);
  }
}

std::ostream &operator<<(std::ostream &os, const Image &image) {
  if (image.contain_mat_) {
    os << "Image device_id:" << static_cast<int>(image.camera_id_)
       << " device_id_str: " << image.camera_id_str()
       << " ts:" << std::setprecision(FLAGS_double_show_precision)
       << image.timestamp_ << " size:" << image.mat_.size();
  } else {
    os << "Image not inited.";
  }
  return os;
}

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
