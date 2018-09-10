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

#include "cybertron/transport/qos/qos_profile_conf.h"
#include "cybertron/tf2_cybertron/transform_listener.h"
#include "cybertron/cybertron.h"

namespace apollo {
namespace cybertron {
namespace tf2_cybertron {

TransformListener::TransformListener(tf2::BufferCore& buffer, bool spin_thread)
    : buffer_(buffer) {
  // if (spin_thread)
  //   initWithThread();
  // else
  //   Init();
  (void)spin_thread;
  std::string node_name = "transform_listener_" +
                          std::to_string(cybertron::Time::Now().ToNanosecond());
  node_ = cybertron::CreateNode(node_name);
  // message_subscriber_tf_ =
  // node_->CreateReader<adu::common::TransformStampeds>(
  //                 "/tf",
  //                 std::bind(&TransformListener::SubscriptionCallback, this,
  //                 std::placeholders::_1),
  //                 qos_profile);

  // message_subscriber_tf_static_ =
  // node_->CreateReader<adu::common::TransformStampeds>(
  //                 "/tf_static",
  //                 std::bind(&TransformListener::StaticSubscriptionCallback,
  //                 this, std::placeholders::_1),
  //                 qos_profile);
  
  apollo::cybertron::proto::RoleAttributes attr;
  attr.set_channel_name("/tf");
  message_subscriber_tf_ =
      node_->CreateReader<adu::common::TransformStampeds>(
          attr,
          [&](const std::shared_ptr<adu::common::TransformStampeds>&
                  msg_evt) { SubscriptionCallbackImpl(msg_evt, false); });

  apollo::cybertron::proto::RoleAttributes attr_static;
  attr_static.set_channel_name("/tf_static");
  attr_static.mutable_qos_profile()->CopyFrom(transport::QosProfileConf::QOS_PROFILE_TF_STATIC);
  message_subscriber_tf_static_ =
      node_->CreateReader<adu::common::TransformStampeds>(
          attr_static,
          [&](const std::shared_ptr<adu::common::TransformStampeds>&
                  msg_evt) { SubscriptionCallbackImpl(msg_evt, true); });

      // node_->CreateReaderRtps<adu::common::TransformStampeds>(
//      node_->RegisterParamSetAllCallback<adu::common::TransformStampeds>(
//          "/tf_static", [&](const adu::common::TransformStampeds& tf) {
//            auto msg_evt = std::make_shared<adu::common::TransformStampeds>(std::move(tf));
//            SubscriptionCallbackImpl(msg_evt, true);
//          });

  // param_handler_ =
  // cybertron::parameter::ParameterHandler::make_shared(node_);
  // param_handler_->RegisterSetGlobalCallback(
  //     "tf_static",
  //     [&](const cybertron::parameter::Parameter& param){
  //       adu::common::TransformStampeds transforms;
  //       transforms.ParseFromString(param.AsString());
  //       cybertron::Time now = cybertron::Time::Now();
  //       if(now.ToNanosecond() < last_update_.ToNanosecond()){
  //         LOG_INFO << "Detected jump back in time. Clearing TF buffer.";
  //         buffer_.clear();
  //       }
  //       last_update_ = now;
  //       LOG_INFO << "Detected jump back in time. Clearing TF buffer.";

  //       // const tf2_msgs::TFMessage& msg_in =
  //       *(transforms.getConstMessage());
  //       std::string authority = "cybertron_tf";
  //       //transforms.getPublisherName(); // lookup the authority
  //       for (unsigned int i = 0; i < transforms.transforms_size(); i++)
  //       {
  //         try
  //         {
  //           buffer_.setTransform(transforms.transforms(i), authority, true);
  //         }

  //         catch (tf2::TransformException& ex)
  //         {
  //           ///\todo Use error reporting
  //           std::string temp = ex.what();
  //           // LOG_ERROR << "Failure to set recieved transform from %s to %s
  //           with error: %s\n", msg_in.transforms[i].child_frame_id.c_str(),
  //           msg_in.transforms[i].header.frame_id.c_str(), temp.c_str());
  //         }
  //       }
  //     });
  // Init();
}

// TransformListener::TransformListener(tf2::BufferCore& buffer, const
// cybertron::NodeHandle& nh, bool spin_thread)
// : dedicated_listener_thread_(NULL)
// , node_(nh)
// , buffer_(buffer)
// , using_dedicated_thread_(false)
// {
//   if (spin_thread)
//     initWithThread();
//   else
//     Init();
// }

TransformListener::~TransformListener() {
  // using_dedicated_thread_ = false;
  // if (dedicated_listener_thread_)
  // {
  //   dedicated_listener_thread_->join();
  //   delete dedicated_listener_thread_;
  // }
}

void TransformListener::Init() {
  // message_subscriber_tf_ = node_.subscribe<tf2_msgs::TFMessage>("/tf", 100,
  // boost::bind(&TransformListener::SubscriptionCallback, this, _1)); ///\todo
  // magic number
  // message_subscriber_tf_static_ =
  // node_.subscribe<tf2_msgs::TFMessage>("/tf_static", 100,
  // boost::bind(&TransformListener::StaticSubscriptionCallback, this, _1));
  // ///\todo magic number
}

void TransformListener::initWithThread() {
  // using_dedicated_thread_ = true;
  // cybertron::SubscribeOptions ops_tf =
  // cybertron::SubscribeOptions::Create<tf2_msgs::TFMessage>("/tf", 100,
  // boost::bind(&TransformListener::SubscriptionCallback, this, _1),
  // cybertron::VoidPtr(), &tf_message_callback_queue_); ///\todo magic number
  // message_subscriber_tf_ = node_.subscribe(ops_tf);

  // cybertron::SubscribeOptions ops_tf_static =
  // cybertron::SubscribeOptions::Create<tf2_msgs::TFMessage>("/tf_static", 100,
  // boost::bind(&TransformListener::StaticSubscriptionCallback, this, _1),
  // cybertron::VoidPtr(), &tf_message_callback_queue_); ///\todo magic number
  // message_subscriber_tf_static_ = node_.subscribe(ops_tf_static);

  // dedicated_listener_thread_ = new
  // boost::thread(boost::bind(&TransformListener::dedicatedListenerThread,
  // this));

  // Tell the buffer we have a dedicated thread to enable timeouts
  // buffer_.setUsingDedicatedThread(true);
}

void TransformListener::SubscriptionCallback(
    const std::shared_ptr<adu::common::TransformStampeds>& msg_evt) {
  SubscriptionCallbackImpl(msg_evt, false);
}
void TransformListener::StaticSubscriptionCallback(
    const std::shared_ptr<adu::common::TransformStampeds>& msg_evt) {
  SubscriptionCallbackImpl(msg_evt, true);
}

void TransformListener::SubscriptionCallbackImpl(
    const std::shared_ptr<adu::common::TransformStampeds>& msg_evt,
    bool is_static) {
  cybertron::Time now = cybertron::Time::Now();
  std::string authority =
      "cybertron_tf";  // msg_evt.getPublisherName(); // lookup the authority
  if (now.ToNanosecond() < last_update_.ToNanosecond()) {
    AINFO << "Detected jump back in time. Clearing TF buffer.";
    buffer_.clear();
    for (auto &msg : static_msgs_) {
      buffer_.setTransform(msg, authority, true);
    }
  }

  last_update_ = now;

  for (int i = 0; i < msg_evt->transforms_size(); i++) {
    try {
      geometry_msgs::TransformStamped trans_stamped;
      //header
      trans_stamped.header.stamp = msg_evt->transforms(i).header().stamp();
      trans_stamped.header.frame_id = msg_evt->transforms(i).header().frame_id();
      trans_stamped.header.seq = msg_evt->transforms(i).header().sequence_num();

      //child_frame_id
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

      if (is_static){
        static_msgs_.push_back(trans_stamped);
      }
      buffer_.setTransform(trans_stamped, authority, is_static);
    }

    catch (tf2::TransformException& ex) {
      ///\todo Use error reporting
      // std::string temp = ex.what();
      // LOG_ERROR << "Failure to set recieved transform from %s to %s with
      // error: %s\n", msg_in.transforms[i].child_frame_id.c_str(),
      // msg_in.transforms[i].header.frame_id.c_str(), temp.c_str());
    }
  }
};

}
}
}
