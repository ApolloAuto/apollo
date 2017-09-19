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

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <string>

#include "driver.h"

namespace apollo {
namespace drivers {
namespace velodyne {

class DriverNodelet : public nodelet::Nodelet {
 public:
  DriverNodelet() : _runing(false) {}

  ~DriverNodelet() {
    if (_runing) {
      ROS_INFO("shutting down driver thread");
      _runing = false;
      _device_thread->join();
      ROS_INFO("driver thread stopped");
    }
  }

 private:
  virtual void onInit(void);
  virtual void device_poll(void);

  volatile bool _runing;  ///< device thread is running
  boost::shared_ptr<boost::thread> _device_thread;

  boost::shared_ptr<VelodyneDriver> _dvr;  ///< driver implementation class
};

void DriverNodelet::onInit() {
  ROS_INFO("Velodyne driver nodelet init");
  // start the driver
  VelodyneDriver *driver =
      VelodyneDriverFactory::create_driver(getPrivateNodeHandle());
  if (driver == nullptr) {
    ROS_BREAK();
  }
  _dvr.reset(driver);
  _dvr->init(getNodeHandle());
  // spawn device poll thread
  _runing = true;
  _device_thread = boost::shared_ptr<boost::thread>(
      new boost::thread(boost::bind(&DriverNodelet::device_poll, this)));
}

/** @brief Device poll thread main loop. */
void DriverNodelet::device_poll() {
  while (ros::ok()) {
    // poll device until end of file
    _runing = _dvr->poll();

    if (!_runing) {
      ROS_ERROR("Running false, stop poll!");
      break;
    }
  }

  _runing = false;
}

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo

// Register this plugin with pluginlib.  Names must match nodelet_velodyne.xml.
//
// parameters are: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(velodyne_driver, DriverNodelet,
                        apollo::drivers::velodyne::DriverNodelet,
                        nodelet::Nodelet);
