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

#ifndef MODULES_DRIVERS_CAMERA_NODES_CAMERA_NODELET_H_
#define MODULES_DRIVERS_CAMERA_NODES_CAMERA_NODELET_H_

#include <boost/thread.hpp>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

// #include "modules/common/log.h"
// #include "modules/common/util/file.h"
// #include "modules/drivers/camera/common/camera_gflags.h"
#include "modules/drivers/camera/nodes/usb_cam_wrapper.h"

namespace apollo {
namespace drivers {
namespace camera {

class CameraNodelet {
 public:
  CameraNodelet();
  ~CameraNodelet();

  void OnInit();

 private:
  boost::shared_ptr<UsbCamWrapper> usb_cam_wrapper_ = nullptr;
  boost::shared_ptr<boost::thread> device_thread_ = nullptr;
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
};

}  // namespace camera
}  // namespace drivers
}  // namespace apollo

// Register this plugin with pluginlib.  Names must match nodelets.xml.
//
// parameters: package, class name, class type, base class type
// PLUGINLIB_DECLARE_CLASS(camera, UsbCamNodelet,
//                        ::apollo::drivers::camera::CameraNodelet,
//                         nodelet::Nodelet);

#endif // MODULES_DRIVERS_CAMERA_NODES_CAMERA_NODELET_H_
