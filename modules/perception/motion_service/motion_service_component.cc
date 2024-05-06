/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/motion_service/motion_service_component.h"

#include <limits>
#include <string>
#include <unordered_map>

#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

#include "cyber/common/log.h"
#include "modules/common/math/math_utils.h"
#include "modules/perception/common/algorithm/sensor_manager/sensor_manager.h"

namespace apollo {
namespace perception {
namespace camera {

bool MotionServiceComponent::Init() {
  AINFO << "start to init MotionService.";
  vehicle_planemotion_ = new PlaneMotion(motion_buffer_size_);

  // the macro READ_CONF would return cyber::FAIL if config not exists
  apollo::perception::onboard::MotionServiceComponent motion_service_param;
  if (!GetProtoConfig(&motion_service_param)) {
    AINFO << "load lane detection component proto param failed";
    return false;
  }

  std::string camera_names_str = motion_service_param.camera_names();
  boost::algorithm::split(camera_names_, camera_names_str,
                          boost::algorithm::is_any_of(","));

  std::string input_camera_channel_names_str =
      motion_service_param.input_camera_channel_names();
  boost::algorithm::split(input_camera_channel_names_,
                          input_camera_channel_names_str,
                          boost::algorithm::is_any_of(","));
  if (input_camera_channel_names_.size() != camera_names_.size()) {
    AERROR << "wrong input_camera_channel_names_.size(): "
           << input_camera_channel_names_.size();
    return cyber::FAIL;
  }

  // initialize image listener
  const std::string &channel_name_img = input_camera_channel_names_[0];
  const std::string &camera_name = camera_names_[0];
  std::function<void(const ImageMsgType &)> camera_callback =
      std::bind(&MotionServiceComponent::OnReceiveImage, this,
                std::placeholders::_1, camera_name);
  auto camera_reader = node_->CreateReader(channel_name_img, camera_callback);

  // initialize localization listener
  const std::string &channel_name_local =
      motion_service_param.input_localization_channel_name();
  std::function<void(const LocalizationMsgType &)> localization_callback =
      std::bind(&MotionServiceComponent::OnLocalization, this,
                std::placeholders::_1);
  auto localization_reader =
      node_->CreateReader(channel_name_local, localization_callback);

  // initialize writer to output channel
  writer_ = node_->CreateWriter<MotionService>(
      motion_service_param.output_topic_channel_name());
  AINFO << "init MotionService success.";
  return true;
}

// On receiving image input, just need to record its timestamp
void MotionServiceComponent::OnReceiveImage(const ImageMsgType &message,
                                            const std::string &camera_name) {
  std::lock_guard<std::mutex> lock(mutex_);
  const double curr_timestamp = message->measurement_time() + timestamp_offset_;
  ADEBUG << "image received: camera_name: " << camera_name
         << " image ts: " << curr_timestamp;
  camera_timestamp_ = curr_timestamp;
}

// On reveiving localization input, register it to camera timestamp,
// compute motion between camera time stamps
void MotionServiceComponent::OnLocalization(
    const LocalizationMsgType &message) {
  std::lock_guard<std::mutex> lock(mutex_);
  ADEBUG << "localization received: localization ts: "
         << message->measurement_time();
  const auto &velocity = message->pose().linear_velocity();
  // Get base::VehicleStatus
  base::VehicleStatus vehicle_status;
  float velx = static_cast<float>(velocity.x());
  float vely = static_cast<float>(velocity.y());
  float velz = static_cast<float>(velocity.z());
  vehicle_status.velocity =
      static_cast<float>(sqrt(velx * velx + vely * vely + velz * velz));
  vehicle_status.velocity_x = velx;
  vehicle_status.velocity_y = vely;
  vehicle_status.velocity_z = velz;
  double timestamp_diff = 0;
  if (!start_flag_) {
    start_flag_ = true;
    vehicle_status.yaw_rate = 0;
    timestamp_diff = 0;
    vehicle_status.time_d = 0;
    vehicle_status.time_ts = 0;

  } else {
    vehicle_status.roll_rate =
        static_cast<float>(message->pose().angular_velocity_vrf().y());
    vehicle_status.pitch_rate =
        static_cast<float>(-message->pose().angular_velocity_vrf().x());
    vehicle_status.yaw_rate =
        static_cast<float>(message->pose().angular_velocity_vrf().z());
    timestamp_diff = message->measurement_time() - pre_timestamp_;
    vehicle_status.time_d = timestamp_diff;
    vehicle_status.time_ts = message->measurement_time();
  }

  pre_timestamp_ = message->measurement_time();

  // add motion to buffer
  // double camera_timestamp = camera_shared_data_->GetLatestTimestamp();
  double camera_timestamp = 0;
  camera_timestamp = camera_timestamp_;
  ADEBUG << "motion processed for camera timestamp: " << camera_timestamp;

  if (start_flag_) {
    if (std::abs(camera_timestamp - pre_camera_timestamp_) <
        std::numeric_limits<double>::epsilon()) {
      ADEBUG << "Motion_status: accum";
      vehicle_planemotion_->add_new_motion(
          pre_camera_timestamp_, camera_timestamp, PlaneMotion::ACCUM_MOTION,
          &vehicle_status);
    } else if (camera_timestamp > pre_camera_timestamp_) {
      ADEBUG << "Motion_status: accum_push";
      vehicle_planemotion_->add_new_motion(
          pre_camera_timestamp_, camera_timestamp,
          PlaneMotion::ACCUM_PUSH_MOTION, &vehicle_status);
      PublishEvent(camera_timestamp);
    } else {
      ADEBUG << "Motion_status: pop";
      vehicle_planemotion_->add_new_motion(pre_camera_timestamp_,
                                           camera_timestamp, PlaneMotion::RESET,
                                           &vehicle_status);
    }
  }

  pre_camera_timestamp_ = camera_timestamp;
}

// pubulish vehicle status buffer to output channel
// which is at camera timestamp
void MotionServiceComponent::PublishEvent(const double timestamp) {
  // protobuf msg
  std::shared_ptr<apollo::perception::MotionService> motion_service_msg(
      new (std::nothrow) apollo::perception::MotionService);
  apollo::common::Header *header = motion_service_msg->mutable_header();

  // output camera_time when motion service last proceesed
  header->set_timestamp_sec(timestamp);
  header->set_module_name("motion_service");

  // convert vehicle status buffer to output message
  auto motion_buffer = GetMotionBuffer();
  for (int k = static_cast<int>(motion_buffer.size()) - 1; k >= 0; k--) {
    apollo::perception::VehicleStatus *v_status_msg =
        motion_service_msg->add_vehicle_status();
    ConvertVehicleMotionToMsgOut(motion_buffer.at(k), v_status_msg);
  }
  writer_->Write(motion_service_msg);
}

// convert vehicle status buffer to output message
void MotionServiceComponent::ConvertVehicleMotionToMsgOut(
    base::VehicleStatus vs, apollo::perception::VehicleStatus *v_status_msg) {
  v_status_msg->set_roll_rate(vs.roll_rate);
  v_status_msg->set_pitch_rate(vs.pitch_rate);
  v_status_msg->set_yaw_rate(vs.yaw_rate);
  v_status_msg->set_velocity(vs.velocity);
  v_status_msg->set_velocity_x(vs.velocity_x);
  v_status_msg->set_velocity_y(vs.velocity_y);
  v_status_msg->set_velocity_z(vs.velocity_z);
  v_status_msg->set_time_ts(vs.time_ts);
  v_status_msg->set_time_d(vs.time_d);
  apollo::perception::MotionType *motion_msg = v_status_msg->mutable_motion();
  motion_msg->set_m00(vs.motion(0, 0));
  motion_msg->set_m01(vs.motion(0, 1));
  motion_msg->set_m02(vs.motion(0, 2));
  motion_msg->set_m03(vs.motion(0, 3));
  motion_msg->set_m10(vs.motion(1, 0));
  motion_msg->set_m11(vs.motion(1, 1));
  motion_msg->set_m12(vs.motion(1, 2));
  motion_msg->set_m13(vs.motion(1, 3));
  motion_msg->set_m20(vs.motion(2, 0));
  motion_msg->set_m21(vs.motion(2, 1));
  motion_msg->set_m22(vs.motion(2, 2));
  motion_msg->set_m23(vs.motion(2, 3));
  motion_msg->set_m30(vs.motion(3, 0));
  motion_msg->set_m31(vs.motion(3, 1));
  motion_msg->set_m32(vs.motion(3, 2));
  motion_msg->set_m33(vs.motion(3, 3));
}

// load vehicle status buffer from vehicle_planemotion_
base::MotionBuffer MotionServiceComponent::GetMotionBuffer() {
  std::lock_guard<std::mutex> lock(motion_mutex_);
  return vehicle_planemotion_->get_buffer();
}

double MotionServiceComponent::GetLatestTimestamp() {
  return pre_camera_timestamp_;
}

// retrieve vehiclestattus at the closeset cameratimestamp
bool MotionServiceComponent::GetMotionInformation(double timestamp,
                                                  base::VehicleStatus *vs) {
  return vehicle_planemotion_->find_motion_with_timestamp(timestamp, vs);
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
