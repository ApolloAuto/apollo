// Copyright 2016 Baidu Inc. All Rights Reserved.
// @autho: zhengwenchao(zhengwenchao@baidu.com)
// @file: image.h
// @brief: Image Class

#ifndef ADU_PERCEPTION_TRAFFIC_LIGHT_BASE_IMAGE_H
#define ADU_PERCEPTION_TRAFFIC_LIGHT_BASE_IMAGE_H

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
  NARROW_FOCUS = 1,  // 12mm
  SHORT_FOCUS = 2,        // 6mm
  WIDE_FOCUS = 3,   // 2mm
  UNKNOWN = 4,
  CAMERA_ID_COUNT = 5
};

const std::map<int, std::string> CAMERA_ID_TO_STR = {
    {static_cast<int>(LONG_FOCUS), "long_focus_camera(25mm)"},
    {static_cast<int>(NARROW_FOCUS), "narrow_focus_camera(12mm)"},
    {static_cast<int>(SHORT_FOCUS), "short_focus_camera(6mm)"},
    {static_cast<int>(WIDE_FOCUS), "wide_focus_camera(2.1mm)"}
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
  bool init(const double ts, const CameraId device_id, const cv::Mat &mat);

  //@brief init
  //@param [in] ts image's timestamp
  //@param [in] camera id
  //@param [in] image's data
  bool
  init(const double ts, const CameraId device_id, const sensor_msgs::ImageConstPtr &image_data);
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
    return _contain_mat;
  }
  bool contain_image() const {
    return _contain_image;
  }
  void set_ts(double ts) {
    _timestamp = ts;
  }

  void set_device_id(CameraId camera_id) {
    _device_id = camera_id;
  }

  bool generate_mat();
 private:
  bool _contain_image = false;
 public:

 private:
  bool _contain_mat = false;
  double _timestamp = 0.0;   //Image's timestamp
  CameraId _device_id = CameraId::UNKNOWN;   //camera's id
  cv::Mat _mat;         //Image's data
  sensor_msgs::ImageConstPtr _image_data;
  friend std::ostream &operator<<(std::ostream &os, const Image &image);
};

typedef std::shared_ptr<Image> ImageSharedPtr;

}// namespace traffic_light
}// namespace perception
}// namespace apollo

#endif  // ADU_PERCEPTION_TRAFFIC_LIGHT_BASE_IMAGE_H
