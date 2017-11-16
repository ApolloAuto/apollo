// Copyright 2016 Baidu Inc. All Rights Reserved.
// @autho: zhengwenchao(zhengwenchao@baidu.com)
// @file: image.h
// @brief: Image Class

#ifndef MODULES_PERCEPTION_TRAFFIC_LIGHT_BASE_IMAGE_H
#define MODULES_PERCEPTION_TRAFFIC_LIGHT_BASE_IMAGE_H

#include <gflags/gflags.h>
#include <string>
#include <map>
#include <memory>

#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>

namespace apollo {
namespace perception {
namespace traffic_light {

//Camera id
enum CameraId {
  LONG_FOCUS = 0,         // 25mm
  SHORT_FOCUS = 1,        // 6mm
  UNKNOWN = 2,
  CAMERA_ID_COUNT = 3
};

const std::map<int, std::string> kCameraIdToStr = {
    {static_cast<int>(LONG_FOCUS), "long_focus_camera(25mm)"},
    {static_cast<int>(SHORT_FOCUS), "short_focus_camera(6mm)"}
};

//@brief Image loaded from camera.
//       Warning: Image is not Thread Safe.
class Image {
 public:
  //@brief constructor
  Image() = default;

  //@brief init
  //@param [in] ts image's timestamp
  //@param [in] camera id
  //@param [in] image's data
  bool Init(const double &ts, const CameraId &device_id, const cv::Mat &mat);

  //@brief init
  //@param [in] ts image's timestamp
  //@param [in] camera id
  //@param [in] image's data
  bool
  Init(const double &ts, const CameraId &device_id, std::shared_ptr<sensor_msgs::Image> image_data);
  //@brief return image's timestamp
  double ts() const;

  //@brief return image's device_id
  CameraId device_id() const;
  //@brief return image's device_id_str
  std::string device_id_str() const;

  //@brief return image's data
  cv::Mat mat() const;

  cv::Size size() const;
  //@brief cotain image.
  bool contain_mat() const {
    return contain_mat_;
  }
  bool contain_image() const {
    return contain_image_;
  }
  void set_ts(double ts) {
    timestamp_ = ts;
  }

  void set_device_id(CameraId camera_id) {
    device_id_ = camera_id;
  }

  bool GenerateMat();

 private:
  bool contain_image_ = false;
  bool contain_mat_ = false;
  double timestamp_ = 0.0;   //Image's timestamp
  CameraId device_id_ = CameraId::UNKNOWN;   //camera's id
  cv::Mat mat_;         //Image's data
  std::shared_ptr<sensor_msgs::Image> image_data_;
  friend std::ostream &operator<<(std::ostream &os, const Image &image);
};

typedef std::shared_ptr<Image> ImageSharedPtr;

}// namespace traffic_light
}// namespace perception
}// namespace apollo

#endif  // MODULES_PERCEPTION_TRAFFIC_LIGHT_BASE_IMAGE_H
