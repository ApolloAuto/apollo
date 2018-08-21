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

#include "modules/drivers/camera/nodes/usb_cam_wrapper.h"

#include <boost/thread.hpp>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

namespace apollo {
namespace drivers {
namespace camera {

class UsbCamNodelet: public nodelet::Nodelet {
 public:
  UsbCamNodelet() {}
  ~UsbCamNodelet() {
    ROS_INFO("shutting down driver thread");
    if (device_thread_ != nullptr && device_thread_->joinable()) {
      device_thread_->join();
    }
    ROS_INFO("driver thread stopped");
  }

 private:
  virtual void onInit();
  boost::shared_ptr<UsbCamWrapper> usb_cam_ = nullptr;
  boost::shared_ptr<boost::thread> device_thread_ = nullptr;
};

void UsbCamNodelet::onInit() {
  ROS_INFO("Usb cam nodelet init");
  usb_cam_.reset(new UsbCamWrapper(getNodeHandle(), getPrivateNodeHandle()));
  // spawn device poll thread
  device_thread_ = boost::shared_ptr<boost::thread>
        (new boost::thread(boost::bind(&UsbCamWrapper::spin, usb_cam_)));
}

}  // namespace camera
}  // namespace drivers
}  // namespace apollo

// Register this plugin with pluginlib.  Names must match nodelets.xml.
//
// parameters: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(usb_cam, UsbCamNodelet,
                        ::apollo::drivers::camera::UsbCamNodelet,
                         nodelet::Nodelet);
