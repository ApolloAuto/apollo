// Copyright 2016 Baidu Inc. All Rights Reserved.
// @autho: zhengwenchao(zhengwenchao@baidu.com)
// @file: image.cpp
// @brief: Image Class

#include "modules/perception/traffic_light/base/image.h"

#include <iomanip>

#include <gflags/gflags.h>
#include "modules/common/log.h"
#include <cv_bridge/cv_bridge.h>

DEFINE_int32(double_show_precision, 14, "When output a double data, the precision.");

namespace adu {
namespace perception {
namespace traffic_light {

bool Image::init(const double ts, const CameraId device_id, const cv::Mat &mat) {
  _contain_mat = true;
  _contain_image = true;
  _timestamp = ts,
  _device_id = device_id,
  _mat = mat.clone();
  ADEBUG << *this << " init.";
  return true;
}
bool Image::init(const double ts, const CameraId device_id,
                 const sensor_msgs::ImageConstPtr &image_data) {
  _contain_mat = false;
  _contain_image = true;
  _timestamp = ts,
  _device_id = device_id,
  _image_data = image_data;
  ADEBUG << *this << " init.";
  return true;
}

double Image::ts() const {
  return _timestamp;
}

CameraId Image::device_id() const {
  return _device_id;
}

std::string Image::device_id_str() const {
  int device_id = static_cast<int>(_device_id);
  if (CAMERA_ID_TO_STR.find(device_id) == CAMERA_ID_TO_STR.end()) {
    return "unkown device(camera)";
  }

  return CAMERA_ID_TO_STR.at(device_id);
}
bool Image::generate_mat() {
  if (!_contain_mat) {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      if (_image_data->encoding.compare("rgb8") == 0) {
        cv_ptr = cv_bridge::toCvShare(_image_data, "bgr8");
      } else if (_image_data->encoding.compare("8UC3") == 0) {
        cv_ptr = cv_bridge::toCvShare(_image_data, "8UC3");
      } else {
        AERROR << "TLPreprocessorSubnode get unknown image format. "
               << "format:" << _image_data->encoding;
        return false;
      }
      _mat = cv_ptr->image;
      _contain_mat = true;
      AINFO << "Generate done " << _mat.size();
    }
    catch (const cv_bridge::Exception &e) {
      AERROR << "TLPreprocessorSubnode trans msg to cv::Mat failed." << e.what();
      return false;
    }
  }
  return true;
}
cv::Mat Image::mat() const {
  return _mat;
}
cv::Size Image::size() const {
  if (_contain_mat) {
    return _mat.size();
  } else {
    return cv::Size(_image_data->width, _image_data->height);
  }
}

std::ostream &operator<<(std::ostream &os, const Image &image) {

  if (image._contain_mat) {
    os << "Image device_id:" << static_cast<int>(image._device_id)
       << " device_id_str: " << image.device_id_str()
       << " ts:" << std::setprecision(FLAGS_double_show_precision) << image._timestamp
       << " size:" << image._mat.size();
  } else {
    os << "Image not inited.";
  }
  return os;
}

}// namespace traffic_light
}// namespace perception
}// namespace adu
