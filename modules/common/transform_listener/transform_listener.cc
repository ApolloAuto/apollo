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

#include "modules/common/transform_listener/transform_listener.h"

#include <functional>

#include "modules/common/log.h"
#include "modules/common/time/time.h"

namespace apollo {
namespace common {

using apollo::common::time::Clock;
using tf2_msgs::TFMessage;
using std::placeholders::_1;

TransformListener::TransformListener(tf2::BufferCore* buffer,
                                     ros::NodeHandle* nh, bool spin_thread)
    : node_(nh),
      buffer_(buffer),
      using_dedicated_thread_(spin_thread),
      last_update_(0.0) {
  if (node_ == nullptr) {
    // There is no node handle when ROS is not enabled.
    return;
  }
  if (spin_thread) {
    InitWithThread();
  } else {
    Init();
  }
}

TransformListener::~TransformListener() {
  using_dedicated_thread_ = false;
  if (dedicated_listener_thread_) {
    dedicated_listener_thread_->join();
  }
}

void TransformListener::Init() {
  tf_subscriber_ =
      node_->subscribe("/tf", kQueueSize, &TransformListener::TfCallback, this);
  tf_static_subscriber_ = node_->subscribe(
      "/tf_static", kQueueSize, &TransformListener::TfStaticCallback, this);
}

void TransformListener::InitWithThread() {
  using_dedicated_thread_ = true;
  ros::SubscribeOptions ops_tf = ros::SubscribeOptions::create<TFMessage>(
      "/tf", kQueueSize, std::bind(&TransformListener::TfCallback, this, _1),
      ros::VoidPtr(), &tf_message_callback_queue_);
  tf_subscriber_ = node_->subscribe(ops_tf);

  ros::SubscribeOptions ops_tf_static =
      ros::SubscribeOptions::create<TFMessage>(
          "/tf_static", kQueueSize,
          std::bind(&TransformListener::TfStaticCallback, this, _1),
          ros::VoidPtr(), &tf_message_callback_queue_);
  tf_static_subscriber_ = node_->subscribe(ops_tf_static);

  dedicated_listener_thread_.reset(new std::thread(
      std::bind(&TransformListener::DedicatedListenerThread, this)));

  // Tell the buffer we have a dedicated thread to enable timeouts
  buffer_->setUsingDedicatedThread(true);
}

void TransformListener::TfCallback(tf2_msgs::TFMessage::ConstPtr tf) {
  CallbackImpl(tf, false);
}
void TransformListener::TfStaticCallback(
    tf2_msgs::TFMessage::ConstPtr tf_static) {
  CallbackImpl(tf_static, true);
}

void TransformListener::CallbackImpl(tf2_msgs::TFMessage::ConstPtr tf,
                                     bool is_static) {
  double now = Clock::NowInSeconds();
  if (now < last_update_) {
    AWARN << "Detected jump back in time. Clearing TF buffer.";
    buffer_->clear();
  }
  last_update_ = now;
  for (size_t i = 0; i < tf->transforms.size(); i++) {
    try {
      buffer_->setTransform(tf->transforms[i], "tf", is_static);
    } catch (tf2::TransformException& ex) {
      AERROR << "Failure to set received transform from "
             << tf->transforms[i].child_frame_id << " to "
             << tf->transforms[i].header.frame_id
             << " with error: " << ex.what();
    }
  }
}

void TransformListener::DedicatedListenerThread() {
  while (using_dedicated_thread_) {
    tf_message_callback_queue_.callAvailable(ros::WallDuration(0.01));
  }
}

}  // namespace common
}  // namespace apollo
