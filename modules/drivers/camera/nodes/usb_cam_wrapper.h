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

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <pluginlib/class_loader.h>
#include <std_srvs/Empty.h>

#include <string>

#include "modules/drivers/camera/proto/camera_conf.pb.h"
#include "modules/drivers/camera/usb_cam/usb_cam.h"
#include "modules/drivers/proto/sensor_image.pb.h"

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
  UsbCamWrapper(ros::NodeHandle node,
                ros::NodeHandle private_nh,
                CameraConf config);
  virtual ~UsbCamWrapper();
  bool service_start_cap(std_srvs::Empty::Request &req,
                         std_srvs::Empty::Response &res);
  bool service_stop_cap(std_srvs::Empty::Request &req,
                        std_srvs::Empty::Response &res);
  bool take_and_send_image();
  bool spin();

 private:
  // private ROS node handle
  ros::NodeHandle node_;
  ros::NodeHandle priv_node_;

  CameraConf config_;
  // shared image message
  sensor_msgs::Image img_;
  ::apollo::drivers::Image sensor_image_;
  sensor_msgs::CameraInfoPtr cam_info_ = nullptr;
  // image_transport::CameraPublisher image_pub_;

  image_transport::PubLoaderPtr pub_loader_;
  boost::shared_ptr<image_transport::PublisherPlugin> image_pub_plugin_;

  ros::Publisher cam_info_pub_;

  UsbCam cam_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

  ros::ServiceServer service_start_;
  ros::ServiceServer service_stop_;

  ros::Time last_stamp_;
  float frame_warning_interval_;
  float frame_drop_interval_;
};

}  // namespace camera
}  // namespace drivers
}  // namespace apollo

#endif // MODULES_DRIVERS_CAMERA_NODES_USB_CAM_WRAPPER_H_
