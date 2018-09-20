/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/transform/buffer.h"

#include <sstream>
#include "cybertron/cybertron.h"
#include "modules/transform/proto/transform.pb.h"

namespace apollo {
namespace transform {

Buffer::Buffer() : BufferCore() { Init(); }

int Buffer::Init() {
  std::string node_name = "transform_listener_" +
                          std::to_string(cybertron::Time::Now().ToNanosecond());
  node_ = cybertron::CreateNode(node_name);
  apollo::cybertron::proto::RoleAttributes attr;
  attr.set_channel_name("/tf");
  message_subscriber_tf_ =
      node_->CreateReader<apollo::transform::TransformStampeds>(
          attr,
          [&](const std::shared_ptr<const apollo::transform::TransformStampeds>&
                  msg_evt) { SubscriptionCallbackImpl(msg_evt, false); });

  apollo::cybertron::proto::RoleAttributes attr_static;
  attr_static.set_channel_name("/tf_static");
  attr_static.mutable_qos_profile()->CopyFrom(
      apollo::cybertron::transport::QosProfileConf::QOS_PROFILE_TF_STATIC);
  message_subscriber_tf_static_ =
      node_->CreateReader<apollo::transform::TransformStampeds>(
          attr_static,
          [&](const std::shared_ptr<apollo::transform::TransformStampeds>&
                  msg_evt) { SubscriptionCallbackImpl(msg_evt, true); });

  return cybertron::SUCC;
}

void Buffer::SubscriptionCallback(
    const std::shared_ptr<const apollo::transform::TransformStampeds>&
        msg_evt) {
  SubscriptionCallbackImpl(msg_evt, false);
}

void Buffer::StaticSubscriptionCallback(
    const std::shared_ptr<const apollo::transform::TransformStampeds>&
        msg_evt) {
  SubscriptionCallbackImpl(msg_evt, true);
}

void Buffer::SubscriptionCallbackImpl(
    const std::shared_ptr<const apollo::transform::TransformStampeds>& msg_evt,
    bool is_static) {
  cybertron::Time now = cybertron::Time::Now();
  std::string authority =
      "cybertron_tf";  // msg_evt.getPublisherName(); // lookup the authority
  if (now.ToNanosecond() < last_update_.ToNanosecond()) {
    AINFO << "Detected jump back in time. Clearing TF buffer.";
    clear();
    // cache static transform stamped again.
    for (auto& msg : static_msgs_) {
      setTransform(msg, authority, true);
    }
  }
  last_update_ = now;

  for (int i = 0; i < msg_evt->transforms_size(); i++) {
    try {
      geometry_msgs::TransformStamped trans_stamped;

      // header
      trans_stamped.header.stamp =
          msg_evt->transforms(i).header().timestamp_sec() * 1e9;
      trans_stamped.header.frame_id =
          msg_evt->transforms(i).header().frame_id();
      trans_stamped.header.seq = msg_evt->transforms(i).header().sequence_num();

      // child_frame_id
      trans_stamped.child_frame_id = msg_evt->transforms(i).child_frame_id();

      // translation
      trans_stamped.transform.translation.x =
          msg_evt->transforms(i).transform().translation().x();
      trans_stamped.transform.translation.y =
          msg_evt->transforms(i).transform().translation().y();
      trans_stamped.transform.translation.z =
          msg_evt->transforms(i).transform().translation().z();

      // rotation
      trans_stamped.transform.rotation.x =
          msg_evt->transforms(i).transform().rotation().qx();
      trans_stamped.transform.rotation.y =
          msg_evt->transforms(i).transform().rotation().qy();
      trans_stamped.transform.rotation.z =
          msg_evt->transforms(i).transform().rotation().qz();
      trans_stamped.transform.rotation.w =
          msg_evt->transforms(i).transform().rotation().qw();

      if (is_static) {
        static_msgs_.push_back(trans_stamped);
      }
      setTransform(trans_stamped, authority, is_static);
    }

    catch (tf2::TransformException& ex) {
      std::string temp = ex.what();
      AERROR << "Failure to set recieved transform:" << temp.c_str();
    }
  }
};

void Buffer::TF2MsgToCyber(
    const geometry_msgs::TransformStamped& tf2_trans_stamped,
    apollo::transform::TransformStamped& trans_stamped) const {
  // header
  trans_stamped.mutable_header()->set_timestamp_sec(
      static_cast<double>(tf2_trans_stamped.header.stamp) / 1e9);
  trans_stamped.mutable_header()->set_frame_id(
      tf2_trans_stamped.header.frame_id);

  // child_frame_id
  trans_stamped.set_child_frame_id(tf2_trans_stamped.child_frame_id);

  // translation
  trans_stamped.mutable_transform()->mutable_translation()->set_x(
      tf2_trans_stamped.transform.translation.x);
  trans_stamped.mutable_transform()->mutable_translation()->set_y(
      tf2_trans_stamped.transform.translation.y);
  trans_stamped.mutable_transform()->mutable_translation()->set_z(
      tf2_trans_stamped.transform.translation.z);

  // rotation
  trans_stamped.mutable_transform()->mutable_rotation()->set_qx(
      tf2_trans_stamped.transform.rotation.x);
  trans_stamped.mutable_transform()->mutable_rotation()->set_qy(
      tf2_trans_stamped.transform.rotation.y);
  trans_stamped.mutable_transform()->mutable_rotation()->set_qz(
      tf2_trans_stamped.transform.rotation.z);
  trans_stamped.mutable_transform()->mutable_rotation()->set_qw(
      tf2_trans_stamped.transform.rotation.w);
}

apollo::transform::TransformStamped Buffer::lookupTransform(
    const std::string& target_frame, const std::string& source_frame,
    const cybertron::Time& time, const float timeout_second) const {
  (void)timeout_second;
  tf2::Time tf2_time(time.ToNanosecond());
  geometry_msgs::TransformStamped tf2_trans_stamped =
      lookupTransform(target_frame, source_frame, tf2_time);
  apollo::transform::TransformStamped trans_stamped;
  TF2MsgToCyber(tf2_trans_stamped, trans_stamped);
  return trans_stamped;
}

apollo::transform::TransformStamped Buffer::lookupTransform(
    const std::string& target_frame, const cybertron::Time& target_time,
    const std::string& source_frame, const cybertron::Time& source_time,
    const std::string& fixed_frame, const float timeout_second) const {
  (void)timeout_second;
  geometry_msgs::TransformStamped tf2_trans_stamped =
      lookupTransform(target_frame, target_time.ToNanosecond(), source_frame,
                      source_time.ToNanosecond(), fixed_frame);
  apollo::transform::TransformStamped trans_stamped;
  TF2MsgToCyber(tf2_trans_stamped, trans_stamped);
  return trans_stamped;
}

bool Buffer::canTransform(const std::string& target_frame,
                          const std::string& source_frame,
                          const cybertron::Time& time,
                          const float timeout_second,
                          std::string* errstr) const {
  uint64_t timeout_ns = timeout_second * 1000000000;
  uint64_t start_time = cybertron::Time::Now().ToNanosecond();
  while (
      cybertron::Time::Now().ToNanosecond() < start_time + timeout_ns &&
      !canTransform(target_frame, source_frame, time.ToNanosecond(), errstr) &&
      !cybertron::IsShutdown())  // Make sure we haven't been stopped (won't
                                 // work for
                                 // pytf)
  {
    usleep(3000);
  }
  bool retval =
      canTransform(target_frame, source_frame, time.ToNanosecond(), errstr);
  // conditionally_append_timeout_info(errstr, start_time, timeout);
  return retval;
}

bool Buffer::canTransform(const std::string& target_frame,
                          const cybertron::Time& target_time,
                          const std::string& source_frame,
                          const cybertron::Time& source_time,
                          const std::string& fixed_frame,
                          const float timeout_second,
                          std::string* errstr) const {
  // poll for transform if timeout is set
  uint64_t timeout_ns = timeout_second * 1000000000;
  uint64_t start_time = cybertron::Time::Now().ToNanosecond();
  while (cybertron::Time::Now().ToNanosecond() < start_time + timeout_ns &&
         !canTransform(target_frame, target_time.ToNanosecond(), source_frame,
                       source_time.ToNanosecond(),
                       fixed_frame) &&
         !cybertron::IsShutdown()) {  // Make sure we haven't been stopped
    usleep(3000);
  }
  bool retval =
      canTransform(target_frame, target_time.ToNanosecond(), source_frame,
                   source_time.ToNanosecond(), fixed_frame, errstr);
  return retval;
}

}  // namespace transform
}  // namespace apollo
