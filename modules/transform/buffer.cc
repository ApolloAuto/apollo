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

#include "absl/strings/str_cat.h"

#include "cyber/cyber.h"
#include "cyber/time/clock.h"
#include "modules/common/adapters/adapter_gflags.h"

using Time = ::apollo::cyber::Time;
using Clock = ::apollo::cyber::Clock;

namespace {
constexpr float kSecondToNanoFactor = 1e9f;
constexpr uint64_t kMilliToNanoFactor = 1e6;
}  // namespace

namespace apollo {
namespace transform {

Buffer::Buffer() : BufferCore() { Init(); }

int Buffer::Init() {
  const std::string node_name =
      absl::StrCat("transform_listener_", Time::Now().ToNanosecond());
  node_ = cyber::CreateNode(node_name);
  apollo::cyber::proto::RoleAttributes attr;
  attr.set_channel_name("/tf");
  message_subscriber_tf_ = node_->CreateReader<TransformStampeds>(
      attr, [&](const std::shared_ptr<const TransformStampeds>& msg_evt) {
        SubscriptionCallbackImpl(msg_evt, false);
      });

  apollo::cyber::proto::RoleAttributes attr_static;
  attr_static.set_channel_name(FLAGS_tf_static_topic);
  attr_static.mutable_qos_profile()->CopyFrom(
      apollo::cyber::transport::QosProfileConf::QOS_PROFILE_TF_STATIC);
  message_subscriber_tf_static_ = node_->CreateReader<TransformStampeds>(
      attr_static, [&](const std::shared_ptr<TransformStampeds>& msg_evt) {
        SubscriptionCallbackImpl(msg_evt, true);
      });

  return cyber::SUCC;
}

void Buffer::SubscriptionCallback(
    const std::shared_ptr<const TransformStampeds>& msg_evt) {
  SubscriptionCallbackImpl(msg_evt, false);
}

void Buffer::StaticSubscriptionCallback(
    const std::shared_ptr<const TransformStampeds>& msg_evt) {
  SubscriptionCallbackImpl(msg_evt, true);
}

void Buffer::SubscriptionCallbackImpl(
    const std::shared_ptr<const TransformStampeds>& msg_evt, bool is_static) {
  cyber::Time now = Clock::Now();
  std::string authority =
      "cyber_tf";  // msg_evt.getPublisherName(); // lookup the authority
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
      const auto& header = msg_evt->transforms(i).header();
      trans_stamped.header.stamp =
          static_cast<uint64_t>(header.timestamp_sec() * kSecondToNanoFactor);
      trans_stamped.header.frame_id = header.frame_id();
      trans_stamped.header.seq = header.sequence_num();

      // child_frame_id
      trans_stamped.child_frame_id = msg_evt->transforms(i).child_frame_id();

      // translation
      const auto& transform = msg_evt->transforms(i).transform();
      trans_stamped.transform.translation.x = transform.translation().x();
      trans_stamped.transform.translation.y = transform.translation().y();
      trans_stamped.transform.translation.z = transform.translation().z();

      // rotation
      trans_stamped.transform.rotation.x = transform.rotation().qx();
      trans_stamped.transform.rotation.y = transform.rotation().qy();
      trans_stamped.transform.rotation.z = transform.rotation().qz();
      trans_stamped.transform.rotation.w = transform.rotation().qw();

      if (is_static) {
        static_msgs_.push_back(trans_stamped);
      }
      setTransform(trans_stamped, authority, is_static);
    } catch (tf2::TransformException& ex) {
      std::string temp = ex.what();
      AERROR << "Failure to set received transform:" << temp.c_str();
    }
  }
}

bool Buffer::GetLatestStaticTF(const std::string& frame_id,
                               const std::string& child_frame_id,
                               TransformStamped* tf) {
  for (auto reverse_iter = static_msgs_.rbegin();
       reverse_iter != static_msgs_.rend(); ++reverse_iter) {
    if ((*reverse_iter).header.frame_id == frame_id &&
        (*reverse_iter).child_frame_id == child_frame_id) {
      TF2MsgToCyber((*reverse_iter), (*tf));
      return true;
    }
  }
  return false;
}

void Buffer::TF2MsgToCyber(
    const geometry_msgs::TransformStamped& tf2_trans_stamped,
    TransformStamped& trans_stamped) const {
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

TransformStamped Buffer::lookupTransform(const std::string& target_frame,
                                         const std::string& source_frame,
                                         const cyber::Time& time,
                                         const float timeout_second) const {
  tf2::Time tf2_time(time.ToNanosecond());
  geometry_msgs::TransformStamped tf2_trans_stamped =
      tf2::BufferCore::lookupTransform(target_frame, source_frame, tf2_time);
  TransformStamped trans_stamped;
  TF2MsgToCyber(tf2_trans_stamped, trans_stamped);
  return trans_stamped;
}

TransformStamped Buffer::lookupTransform(const std::string& target_frame,
                                         const cyber::Time& target_time,
                                         const std::string& source_frame,
                                         const cyber::Time& source_time,
                                         const std::string& fixed_frame,
                                         const float timeout_second) const {
  geometry_msgs::TransformStamped tf2_trans_stamped =
      tf2::BufferCore::lookupTransform(target_frame, target_time.ToNanosecond(),
                                       source_frame, source_time.ToNanosecond(),
                                       fixed_frame);
  TransformStamped trans_stamped;
  TF2MsgToCyber(tf2_trans_stamped, trans_stamped);
  return trans_stamped;
}

bool Buffer::canTransform(const std::string& target_frame,
                          const std::string& source_frame,
                          const cyber::Time& time, const float timeout_second,
                          std::string* errstr) const {
  uint64_t timeout_ns =
      static_cast<uint64_t>(timeout_second * kSecondToNanoFactor);
  uint64_t start_time = Clock::Now().ToNanosecond();  // time.ToNanosecond();
  while (Clock::Now().ToNanosecond() < start_time + timeout_ns &&
         !cyber::IsShutdown()) {
    errstr->clear();
    bool retval = tf2::BufferCore::canTransform(target_frame, source_frame,
                                                time.ToNanosecond(), errstr);
    if (retval) {
      return true;
    } else {
      if (!cyber::common::GlobalData::Instance()->IsRealityMode()) {
        break;
      }
      const int sleep_time_ms = 3;
      AWARN << "BufferCore::canTransform failed: " << *errstr;
      std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));
    }
  }
  *errstr = *errstr + ":timeout";
  return false;
}

bool Buffer::canTransform(const std::string& target_frame,
                          const cyber::Time& target_time,
                          const std::string& source_frame,
                          const cyber::Time& source_time,
                          const std::string& fixed_frame,
                          const float timeout_second,
                          std::string* errstr) const {
  // poll for transform if timeout is set
  uint64_t timeout_ns =
      static_cast<uint64_t>(timeout_second * kSecondToNanoFactor);
  uint64_t start_time = Clock::Now().ToNanosecond();
  while (Clock::Now().ToNanosecond() < start_time + timeout_ns &&
         !cyber::IsShutdown()) {  // Make sure we haven't been stopped
    errstr->clear();
    bool retval = tf2::BufferCore::canTransform(
        target_frame, target_time.ToNanosecond(), source_frame,
        source_time.ToNanosecond(), fixed_frame, errstr);
    if (retval) {
      return true;
    } else {
      const int sleep_time_ms = 3;
      AWARN << "BufferCore::canTransform failed: " << *errstr;
      std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));
      if (!cyber::common::GlobalData::Instance()->IsRealityMode()) {
        Clock::SetNow(Time(Clock::Now().ToNanosecond() +
                           sleep_time_ms * kMilliToNanoFactor));
      }
    }
  }
  *errstr = *errstr + ":timeout";
  return false;
}

}  // namespace transform
}  // namespace apollo
