/******************************************************************************
 * Modification Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include <time.h>   /* for clock_gettime */

#include "image_transport/camera_common.h"
#include "image_transport/publisher_plugin.h"

#include "modules/common/log.h"
#include "modules/common/util/file.h"
#include "modules/drivers/camera/common/camera_gflags.h"
#include "modules/drivers/proto/sensor_image.pb.h"

#include "modules/drivers/camera/nodes/usb_cam_wrapper.h"

namespace apollo {
namespace drivers {
namespace camera {

UsbCamWrapper::UsbCamWrapper(ros::NodeHandle node, ros::NodeHandle private_nh) :
    node_(node), priv_node_(private_nh), last_stamp_(0) {
  if (!::apollo::common::util::GetProtoFromFile(FLAGS_camera_config_file,
                                                &config_)) {
    AERROR << "Unable to load camera conf file: " << FLAGS_camera_config_file;
    return;
  }

  // pb
  // http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html
  sensor_image_.mutable_header()->set_camera_timestamp(img_.header.stamp.toSec());
  sensor_image_.set_frame_id(img_.header.frame_id);
  // TODO(all) sensor_image_.set_measurement_time();
  sensor_image_.set_height(img_.height);  // image height, that is, number of rows
  sensor_image_.set_width(img_.width);  // image width, that is, number of columns
  sensor_image_.set_encoding(img_.encoding);
  sensor_image_.set_step(img_.step);  // Full row length in bytes
  // actual matrix data, size is (step * rows)
  size_t data_length = img_.step * img_.height;
  std::string image_data;
  image_data.assign(reinterpret_cast<const char *>(img_.data.data()), data_length);
  sensor_image_.set_data(image_data);

  cinfo_.reset(new camera_info_manager::CameraInfoManager(
      node_, config_.camera_name(), config_.camera_info_url()));

  // default 3000 ms


  // Warning when diff with last > 1.5* interval
  frame_warning_interval_ = 1.5 / config_.frame_rate();
  // now max fps 30, we use a appox time 0.9 to drop image.
  frame_drop_interval_ = 0.9 / config_.frame_rate();

  // advertise the main image topic
  // image_transport::ImageTransport it(node_);
  // image_pub_ = it.advertiseCamera(topic_name_, 1);
  // Load transport publish plugin
  std::string image_topic = node_.resolveName(config_.topic_name());
  pub_loader_ = boost::make_shared<image_transport::PubLoader>(
      "image_transport", "image_transport::PublisherPlugin");
  std::string lookup_name = image_transport::PublisherPlugin::getLookupName(
      std::string("raw"));
  image_pub_plugin_ = pub_loader_->createInstance(lookup_name);
  if (image_pub_plugin_ != nullptr) {
    image_pub_plugin_->advertise(node_, image_topic, 1,
                                 image_transport::SubscriberStatusCallback(),
           image_transport::SubscriberStatusCallback(), ros::VoidPtr(), false);
  } else {
    AERROR << "create image publish plugin error. lookup_name: " << lookup_name;
    node_.shutdown();
    return;
  }

  // camera info publish
  std::string cam_info_topic =
      image_transport::getCameraInfoTopic(image_topic);
  cam_info_pub_ = node_.advertise<sensor_msgs::CameraInfo>(
      cam_info_topic, 1, ros::SubscriberStatusCallback(),
      ros::SubscriberStatusCallback(), ros::VoidPtr(), false);

  // create Services
  service_start_ = node_.advertiseService(
      "start_capture", &UsbCamWrapper::service_start_cap, this);
  service_stop_ = node_.advertiseService(
      "stop_capture", &UsbCamWrapper::service_stop_cap, this);

  // check for default camera info
  if (!cinfo_->isCalibrated()) {
    cinfo_->setCameraName(config_.video_device());
    sensor_msgs::CameraInfo camera_info;
    camera_info.header.frame_id = img_.header.frame_id;
    camera_info.width = config_.image_width();
    camera_info.height = config_.image_height();
    cinfo_->setCameraInfo(camera_info);
  }
  // get the camera basical infomation
  cam_info_.reset(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));

  AINFO << "Starting '" << config_.camera_name() << "' (" << config_.video_device()
      << ") at " << config_.image_width() << "x" << config_.image_height()
      << " via " << config_.io_method()  << " (" << config_.pixel_format()
      << ") at %" << config_.frame_rate() << " FPS";

  // set the IO method
  UsbCam::io_method io_method =
      UsbCam::io_method_from_string(config_.io_method());

  if (io_method == UsbCam::IO_METHOD_UNKNOWN) {
    AFATAL << "Unknown IO method '" << config_.io_method() << "'";
    node_.shutdown();
    return;
  }

  // set the pixel format
  UsbCam::pixel_format pixel_format = UsbCam::pixel_format_from_string(
      config_.pixel_format());

  if (pixel_format == UsbCam::PIXEL_FORMAT_UNKNOWN) {
    AFATAL << "Unknown pixel format '" << config_.pixel_format() << "'";
    node_.shutdown();
    return;
  }

  // start the camera
  cam_.start(config_.video_device(),
             io_method, pixel_format,
             config_.image_width(), config_.image_height(),
             config_.frame_rate());

  // set camera parameters
  if (config_.brightness() >= 0) {
    cam_.set_v4l_parameter("brightness", config_.brightness());
  }

  if (config_.contrast() >= 0) {
    cam_.set_v4l_parameter("contrast", config_.contrast());
  }

  if (config_.saturation() >= 0) {
    cam_.set_v4l_parameter("saturation", config_.saturation());
  }

  if (config_.sharpness() >= 0) {
    cam_.set_v4l_parameter("sharpness", config_.sharpness());
  }

  if (config_.gain() >= 0) {
    cam_.set_v4l_parameter("gain", config_.gain());
  }

  // check auto white balance
  if (config_.auto_white_balance()) {
    cam_.set_v4l_parameter("white_balance_temperature_auto", 1);
  } else {
    cam_.set_v4l_parameter("white_balance_temperature_auto", 0);
    cam_.set_v4l_parameter("white_balance_temperature", config_.white_balance());
  }

  // check auto exposure
  if (!config_.autoexposure()) {
    // turn down exposure control (from max of 3)
    cam_.set_v4l_parameter("exposure_auto", 1);
    // change the exposure level
    cam_.set_v4l_parameter("exposure_absolute", config_.exposure());
  }

  // check auto focus
  if (config_.autofocus()) {
    cam_.set_auto_focus(1);
    cam_.set_v4l_parameter("focus_auto", 1);
  } else {
    cam_.set_v4l_parameter("focus_auto", 0);

    if (config_.focus() >= 0) {
      cam_.set_v4l_parameter("focus_absolute", config_.focus());
    }
  }

  // trigger enable
  int trigger_ret = cam_.trigger_enable(config_.trigger_fps(),
                                        config_.trigger_internal());
  if (0 != trigger_ret) {
    AWARN << "Camera trigger Fail ret: " << trigger_ret;
    // node_.shutdown();
    // return;
  }
}

UsbCamWrapper::~UsbCamWrapper() {
  cam_.shutdown();
}

bool UsbCamWrapper::service_start_cap(std_srvs::Empty::Request& req,
                                      std_srvs::Empty::Response& res) {
  cam_.start_capturing();
  return true;
}

bool UsbCamWrapper::service_stop_cap(std_srvs::Empty::Request& req,
                                     std_srvs::Empty::Response& res) {
  cam_.stop_capturing();
  return true;
}

bool UsbCamWrapper::take_and_send_image() {
  // grab the image
  bool get_new_image = cam_.grab_image(&img_, config_.camera_timeout());

  if (!get_new_image) {
    return false;
  }

  // grab the camera info
  // cam_info_ = sensor_msgs::CameraInfo(cinfo_->getCameraInfo());
  cam_info_->header.frame_id = img_.header.frame_id;
  cam_info_->header.stamp = img_.header.stamp;

  if (last_stamp_ == ros::Time(0)) {
    last_stamp_ = img_.header.stamp;
  } else {
    auto diff = (img_.header.stamp - last_stamp_).toSec();
    // drop image by frame_rate
    if (diff < frame_drop_interval_) {
      ROS_INFO_STREAM("drop image:" << img_.header.stamp);
      return true;
    }
    if (frame_warning_interval_ < diff) {
      ROS_WARN_STREAM("stamp jump.last stamp:" << last_stamp_
          << " current stamp:" << img_.header.stamp);
    }
    last_stamp_ = img_.header.stamp;
  }

  // publish the image
  image_pub_plugin_->publish(img_);
  cam_info_pub_.publish(cam_info_);

  return true;
}

bool UsbCamWrapper::spin() {
  // spin loop rate should be in accord with the trigger frequence
  ros::Duration loop_interval(config_.spin_interval());

  while (node_.ok()) {
    if (cam_.is_capturing()) {
      if (!take_and_send_image()) {
        AERROR << "USB camera did not respond in time.";
      }
    }
    // ros::spinOnce();
    loop_interval.sleep();
  }
  return true;
}

}  // namespace camera
}  // namespace drivers
}  // namespace apollo
