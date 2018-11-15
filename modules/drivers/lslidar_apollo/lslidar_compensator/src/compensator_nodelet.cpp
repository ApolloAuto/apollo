/***************************************************************************
Copyright 2018 The Apollo Authors. All Rights Reserved                     /
                                                                            /
Licensed under the Apache License, Version 2.0 (the "License");             /
you may not use this file except in compliance with the License.            /
You may obtain a copy of the License at                                     /
                                                                            /
    http://www.apache.org/licenses/LICENSE-2.0                              /
                                                                            /
Unless required by applicable law or agreed to in writing, software         /
distributed under the License is distributed on an "AS IS" BASIS,           /
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    /
See the License for the specific language governing permissions and         /
limitations under the License.                                              /
****************************************************************************/

#include <lslidar_compensator/compensator.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace apollo {
namespace drivers {
namespace lslidar_compensator {

class CompensatorNodelet : public nodelet::Nodelet {
 public:
  CompensatorNodelet() {}
  ~CompensatorNodelet() {}

 private:
  virtual void onInit();
  boost::shared_ptr<Compensator> compensator_;
};

/** @brief Nodelet initialization. */
void CompensatorNodelet::onInit() {
  ROS_INFO("Compensator nodelet init");
  compensator_.reset(new Compensator(getNodeHandle(), getPrivateNodeHandle()));
}

} // end namespace lslidar_c16_decoder
}
}

PLUGINLIB_DECLARE_CLASS(lslidar_compensator, CompensatorNodelet,
    apollo::drivers::lslidar_compensator::CompensatorNodelet,
    nodelet::Nodelet);
