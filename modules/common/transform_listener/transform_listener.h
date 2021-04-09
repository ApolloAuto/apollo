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

#ifndef MODULES_COMMON_TRANSFORM_LISTENER_
#define MODULES_COMMON_TRANSFORM_LISTENER_

#include <memory>
#include <thread>

#include "ros/include/ros/callback_queue.h"
#include "ros/include/ros/ros.h"
#include "ros/include/tf2_msgs/TFMessage.h"
#include "ros/include/tf2_ros/buffer.h"

/**
 * @namespace apollo::common
 * @brief apollo::common
 */
namespace apollo {
namespace common {

class TransformListener {
 public:
  TransformListener(tf2::BufferCore* buffer, ros::NodeHandle* nh,
                    bool spin_thread = true);

  ~TransformListener();

  /// Callback function for ros message subscription
  void TfCallback(tf2_msgs::TFMessage::ConstPtr tf);
  void TfStaticCallback(tf2_msgs::TFMessage::ConstPtr tf_static);

 private:
  void Init();
  void InitWithThread();

  void DedicatedListenerThread();

  void CallbackImpl(tf2_msgs::TFMessage::ConstPtr tf, bool is_static);

  ros::CallbackQueue tf_message_callback_queue_;
  std::unique_ptr<std::thread> dedicated_listener_thread_;
  ros::NodeHandle* node_;  // Doesn't own NodeHandle
  ros::Subscriber tf_subscriber_;
  ros::Subscriber tf_static_subscriber_;
  tf2::BufferCore* buffer_;
  bool using_dedicated_thread_;
  double last_update_;

  static constexpr int kQueueSize = 100;
};

}  // namespace common
}  // namespace apollo

#endif  // MODULES_COMMON_TRANSFORM_LISTENER_
