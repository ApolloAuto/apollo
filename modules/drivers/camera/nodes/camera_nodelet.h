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

#include <boost/thread.hpp>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "modules/common/log.h"
#include "modules/drivers/camera/nodes/usb_cam_wrapper.h"

namespace apollo {
namespace drivers {
namespace camera {

class CameraNodelet {
 public:
  CameraNodelet() {}
  ~CameraNodelet() {
    AINFO << "shutting down driver thread";
    if (device_thread_ != nullptr && device_thread_->joinable()) {
      device_thread_->join();
    }
    AINFO << "driver thread stopped";
  }
  virtual void OnInit(void);

 private:
  boost::shared_ptr<UsbCamWrapper> usb_cam_wrapper_ = nullptr;
  boost::shared_ptr<boost::thread> device_thread_ = nullptr;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
};

void CameraNodelet::OnInit() {
  AINFO << "Usb cam nodelet init";
  usb_cam_wrapper_.reset(new UsbCamWrapper(nh_, private_nh_));
  // spawn device poll thread
  device_thread_ = boost::shared_ptr<boost::thread>
        (new boost::thread(boost::bind(&UsbCamWrapper::spin, usb_cam_wrapper_)));
}

}  // namespace camera
}  // namespace drivers
}  // namespace apollo

// Register this plugin with pluginlib.  Names must match nodelets.xml.
//
// parameters: package, class name, class type, base class type
// PLUGINLIB_DECLARE_CLASS(camera, UsbCamNodelet,
//                        ::apollo::drivers::camera::CameraNodelet,
//                         nodelet::Nodelet);
