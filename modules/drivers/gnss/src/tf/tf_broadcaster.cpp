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

#include "tf_broadcaster.h"

namespace apollo {
namespace drivers {
namespace gnss {

void TFBroadcaster::init() {
  _nh.param("odometry_topic", _odometry_topic,
            std::string("/apollo/sensor/gnss/odometry"));
  _nh.param("frame_id", _frame_id, std::string("world"));
  _nh.param("child_frame_id", _child_frame_id, std::string("novatel"));

  _odometry_sub = _nh.subscribe(_odometry_topic, 256,
                                &TFBroadcaster::odometry_callback, this);
}

void TFBroadcaster::gps_to_transform_stamped(
    const ::apollo::localization::Gps& gps,
    geometry_msgs::TransformStamped* transform) {
  ros::Time time;
  transform->header.stamp = time.fromSec(gps.header().timestamp_sec());
  transform->header.frame_id = _frame_id;
  transform->child_frame_id = _child_frame_id;
  transform->transform.translation.x = gps.localization().position().x();
  transform->transform.translation.y = gps.localization().position().y();
  transform->transform.translation.z = gps.localization().position().z();
  transform->transform.rotation.x = gps.localization().orientation().qx();
  transform->transform.rotation.y = gps.localization().orientation().qy();
  transform->transform.rotation.z = gps.localization().orientation().qz();
  transform->transform.rotation.w = gps.localization().orientation().qw();
}

void TFBroadcaster::odometry_callback(
    const boost::shared_ptr<const ::apollo::localization::Gps>& gps) {
  geometry_msgs::TransformStamped transform;
  gps_to_transform_stamped(*gps, &transform);
  _broadcaster.sendTransform(transform);
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
