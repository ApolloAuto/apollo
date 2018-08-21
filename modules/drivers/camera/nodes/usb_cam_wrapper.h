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

#ifndef MODULES_DRIVERS_CAMERA_NODES_USB_CAM_WRAPPER_H_
#define MODULES_DRIVERS_CAMERA_NODES_USB_CAM_WRAPPER_H_

#include "modules/drivers/camera/src/usb_cam.h"
#include "modules/drivers/proto/sensor_image.pb.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <pluginlib/class_loader.h>
#include <std_srvs/Empty.h>
#include <string>

namespace apollo {
namespace drivers {
namespace camera {

enum TriggerFrequence {
    FPS_10HZ = 10,
    FPS_15HZ = 15,
    FPS_20HZ = 20,
    DEFAULT_FPS = 30
};

class UsbCamWrapper {
 public:
  UsbCamWrapper(ros::NodeHandle node, ros::NodeHandle private_nh);
  virtual ~UsbCamWrapper();
  bool service_start_cap(std_srvs::Empty::Request &req,
                         std_srvs::Empty::Response &res);
  bool service_stop_cap(std_srvs::Empty::Request &req,
                        std_srvs::Empty::Response &res);
  bool take_and_send_image();
  bool spin();

 private:
  // shared image message
  sensor_msgs::Image img_;
  ::apollo::drivers::Image sensor_image_;
  sensor_msgs::CameraInfoPtr cam_info_ = nullptr;
  // image_transport::CameraPublisher image_pub_;

  image_transport::PubLoaderPtr pub_loader_;
  boost::shared_ptr<image_transport::PublisherPlugin> image_pub_plugin_;

  ros::Publisher cam_info_pub_;

  // parameters
  std::string topic_name_;
  std::string video_device_name_;
  std::string io_method_name_;
  std::string pixel_format_name_;
  std::string camera_name_;
  std::string camera_info_url_;

  // std::string start_service_name_, start_service_name_;
  // bool streaming_status_;
  int image_width_ = 0;
  int image_height_ = 0;
  int framerate_ = 0;
  int exposure_ = 0;
  int brightness_ = 0;
  int contrast_ = 0;
  int saturation_ = 0;
  int sharpness_ = 0;
  int focus_ = 0;
  int white_balance_ = 0;
  int gain_ = 0;
  int trigger_internal_ = 0;
  int trigger_fps_ = 0;

  bool autofocus_;
  bool autoexposure_;
  bool auto_white_balance_;

  // usb will be reset when camera timeout
  int cam_timeout_;
  UsbCam cam_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

  ros::ServiceServer service_start_;
  ros::ServiceServer service_stop_;

  // private ROS node handle
  ros::NodeHandle node_;
  ros::NodeHandle priv_node_;

  ros::Time last_stamp_;
  float frame_warning_interval_;
  float frame_drop_interval_;
  float spin_interval_;
  int error_code_;
};

}  // namespace camera
}  // namespace drivers
}  // namespace apollo

#endif /* MODULES_DRIVERS_CAMERA_NODES_USB_CAM_WRAPPER_H_ */
