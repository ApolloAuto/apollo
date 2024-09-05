/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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

#ifndef CYBER_APOLLO_ROS_SINGLE_H_
#define CYBER_APOLLO_ROS_SINGLE_H_

#include <memory>
#include <string>
#include <utility>

#include "cyber/cyber.h"
#include "cyber/ros_bridge/converter_base/converter_base.h"
#include "cyber/ros_bridge/converter_base/message_converter.h"

namespace apollo {
namespace cyber {

template <typename InType0, typename InType1, typename InType2,
          typename InType3, typename OutType0>
class ApolloRosMessageConverter<
    InputTypes<std::shared_ptr<InType0>, std::shared_ptr<InType1>,
               std::shared_ptr<InType2>, std::shared_ptr<InType3>>,
    OutputTypes<std::shared_ptr<OutType0>>> : public MessageConverter {
 public:
  ApolloRosMessageConverter() {}
  ~ApolloRosMessageConverter() override {}

  bool Init() {
    MessageConverter::Init();
    if (!init_.load()) {
      return false;
    }
    if (!LoadConfig(&converter_conf_)) {
      return false;
    }

    apollo::cyber::ReaderConfig reader_cfg_0;
    reader_cfg_0.channel_name = converter_conf_.apollo_channel_name_0();

    apollo::cyber::ReaderConfig reader_cfg_1;
    reader_cfg_1.channel_name = converter_conf_.apollo_channel_name_1();
    auto apollo_reader_1 =
        cyber_node_->template CreateReader<InType1>(reader_cfg_1);

    apollo::cyber::ReaderConfig reader_cfg_2;
    reader_cfg_2.channel_name = converter_conf_.apollo_channel_name_2();
    auto apollo_reader_2 =
        cyber_node_->template CreateReader<InType2>(reader_cfg_2);

    apollo::cyber::ReaderConfig reader_cfg_3;
    reader_cfg_3.channel_name = converter_conf_.apollo_channel_name_3();
    auto apollo_reader_3 =
        cyber_node_->template CreateReader<InType3>(reader_cfg_3);

    std::string ros_topic_name_0 = converter_conf_.ros_topic_name_0();

    auto apollo_blocker_1 =
        blocker::BlockerManager::Instance()->GetBlocker<InType1>(
            reader_cfg_1.channel_name);
    auto apollo_blocker_2 =
        blocker::BlockerManager::Instance()->GetBlocker<InType2>(
            reader_cfg_2.channel_name);
    auto apollo_blocker_3 =
        blocker::BlockerManager::Instance()->GetBlocker<InType3>(
            reader_cfg_3.channel_name);
#ifdef RCLCPP__RCLCPP_HPP_
    auto ros_publisher_0 =
        ros_node_->create_publisher<OutType0>(ros_topic_name_0, 10);
    ros_publishers_.push_back(std::move(ros_publisher_0));
    auto func = [this, ros_publisher_0, apollo_blocker_1, apollo_blocker_2,
                 apollo_blocker_3](const std::shared_ptr<InType0> in) {
#else
    auto func = [this, apollo_blocker_1, apollo_blocker_2,
                 apollo_blocker_3](const std::shared_ptr<InType0> in) {
#endif
      auto out = std::make_shared<OutType0>();
      if (!apollo_blocker_1->IsPublishedEmpty() &&
          !apollo_blocker_2->IsPublishedEmpty() &&
          !apollo_blocker_3->IsPublishedEmpty()) {
        auto msg1 = apollo_blocker_1->GetLatestPublishedPtr();
        auto msg2 = apollo_blocker_2->GetLatestPublishedPtr();
        auto msg3 = apollo_blocker_3->GetLatestPublishedPtr();
        auto in_container =
            InputTypes<std::shared_ptr<InType0>, std::shared_ptr<InType1>,
                       std::shared_ptr<InType2>, std::shared_ptr<InType3>>{
                std::make_tuple(in, msg1, msg2, msg3)};
        auto out_container =
            OutputTypes<std::shared_ptr<OutType0>>{std::make_tuple(out)};
        this->ConvertMsg(in_container, out_container);
#ifdef RCLCPP__RCLCPP_HPP_
        ros_publisher_0->publish(*out);
#endif
      }
    };
    auto apollo_reader_0 =
        cyber_node_->template CreateReader<InType0>(reader_cfg_0, func);
    apollo_readers_.push_back(std::move(apollo_reader_0));
    apollo_readers_.push_back(std::move(apollo_reader_1));
    apollo_readers_.push_back(std::move(apollo_reader_2));
    apollo_readers_.push_back(std::move(apollo_reader_3));

    return true;
  }

 protected:
  virtual bool ConvertMsg(
      InputTypes<std::shared_ptr<InType0>, std::shared_ptr<InType1>,
                 std::shared_ptr<InType2>, std::shared_ptr<InType3>>& input,
      OutputTypes<std::shared_ptr<OutType0>>& output) = 0;
};  // NOLINT

// template <typename InType0, typename InType1, typename InType2,
//           typename InType3, typename OutType0>
// bool ApolloRosMessageConverter<InputTypes<
//                                   std::shared_ptr<InType0>,
//                                   std::shared_ptr<InType1>,
//                                   std::shared_ptr<InType2>,
//                                   std::shared_ptr<InType3>>,
//                                OutputTypes<std::shared_ptr<OutType0>>>::Init()

template <typename InType0, typename InType1, typename InType2,
          typename InType3, typename OutType0, typename OutType1>
class ApolloRosMessageConverter<
    InputTypes<std::shared_ptr<InType0>, std::shared_ptr<InType1>,
               std::shared_ptr<InType2>, std::shared_ptr<InType3>>,
    OutputTypes<std::shared_ptr<OutType0>, std::shared_ptr<OutType1>>>
    : public MessageConverter {
 public:
  ApolloRosMessageConverter() {}
  ~ApolloRosMessageConverter() override {}

  bool Init() override;

 protected:
  virtual bool ConvertMsg(
      InputTypes<std::shared_ptr<InType0>, std::shared_ptr<InType1>,
                 std::shared_ptr<InType2>, std::shared_ptr<InType3>>& input,
      OutputTypes<std::shared_ptr<OutType0>, std::shared_ptr<OutType1>>&
          output) = 0;
};

template <typename InType0, typename InType1, typename InType2,
          typename InType3, typename OutType0, typename OutType1>
bool ApolloRosMessageConverter<
    InputTypes<std::shared_ptr<InType0>, std::shared_ptr<InType1>,
               std::shared_ptr<InType2>, std::shared_ptr<InType3>>,
    OutputTypes<std::shared_ptr<OutType0>, std::shared_ptr<OutType1>>>::Init() {
  MessageConverter::Init();
  if (!init_.load()) {
    return false;
  }
  if (!LoadConfig(&converter_conf_)) {
    return false;
  }

  apollo::cyber::ReaderConfig reader_cfg_0;
  reader_cfg_0.channel_name = converter_conf_.apollo_channel_name_0();

  apollo::cyber::ReaderConfig reader_cfg_1;
  reader_cfg_1.channel_name = converter_conf_.apollo_channel_name_1();
  auto apollo_reader_1 =
      cyber_node_->template CreateReader<InType1>(reader_cfg_1);

  apollo::cyber::ReaderConfig reader_cfg_2;
  reader_cfg_2.channel_name = converter_conf_.apollo_channel_name_2();
  auto apollo_reader_2 =
      cyber_node_->template CreateReader<InType2>(reader_cfg_2);

  apollo::cyber::ReaderConfig reader_cfg_3;
  reader_cfg_3.channel_name = converter_conf_.apollo_channel_name_3();
  auto apollo_reader_3 =
      cyber_node_->template CreateReader<InType3>(reader_cfg_3);

  std::string ros_topic_name_0 = converter_conf_.ros_topic_name_0();
  std::string ros_topic_name_1 = converter_conf_.ros_topic_name_1();

  auto apollo_blocker_1 =
      blocker::BlockerManager::Instance()->GetBlocker<InType1>(
          reader_cfg_1.channel_name);
  auto apollo_blocker_2 =
      blocker::BlockerManager::Instance()->GetBlocker<InType2>(
          reader_cfg_2.channel_name);
  auto apollo_blocker_3 =
      blocker::BlockerManager::Instance()->GetBlocker<InType3>(
          reader_cfg_3.channel_name);

#ifdef RCLCPP__RCLCPP_HPP_
  auto ros_publisher_0 =
      ros_node_->create_publisher<OutType0>(ros_topic_name_0, 10);
  auto ros_publisher_1 =
      ros_node_->create_publisher<OutType1>(ros_topic_name_1, 10);
  ros_publishers_.push_back(std::move(ros_publisher_0));
  ros_publishers_.push_back(std::move(ros_publisher_1));
  auto func = [this, apollo_blocker_1, apollo_blocker_2, apollo_blocker_3,
               ros_publisher_0,
               ros_publisher_1](const std::shared_ptr<InType0> in) {
#else
  auto func = [this, apollo_blocker_1, apollo_blocker_2,
               apollo_blocker_3](const std::shared_ptr<InType0> in) {
#endif
    if (!apollo_blocker_1->IsPublishedEmpty() &&
        !apollo_blocker_2->IsPublishedEmpty() &&
        !apollo_blocker_3->IsPublishedEmpty()) {
      auto msg1 = apollo_blocker_1->GetLatestPublishedPtr();
      auto msg2 = apollo_blocker_2->GetLatestPublishedPtr();
      auto msg3 = apollo_blocker_3->GetLatestPublishedPtr();
      auto out_0 = std::make_shared<OutType0>();
      auto out_1 = std::make_shared<OutType1>();
      auto in_container =
          InputTypes<std::shared_ptr<InType0>, std::shared_ptr<InType1>,
                     std::shared_ptr<InType2>, std::shared_ptr<InType3>>{
              std::make_tuple(in, msg1, msg2, msg3)};
      auto out_container =
          OutputTypes<std::shared_ptr<OutType0>, std::shared_ptr<OutType1>>{
              std::make_tuple(out_0, out_1)};
      this->ConvertMsg(in_container, out_container);
#ifdef RCLCPP__RCLCPP_HPP_
      ros_publisher_0->publish(*out_0);
      ros_publisher_1->publish(*out_1);
#endif
    }
  };
  auto apollo_reader_0 =
      cyber_node_->template CreateReader<InType0>(reader_cfg_0, func);
  apollo_readers_.push_back(std::move(apollo_reader_0));
  apollo_readers_.push_back(std::move(apollo_reader_1));
  apollo_readers_.push_back(std::move(apollo_reader_2));
  apollo_readers_.push_back(std::move(apollo_reader_3));

  return true;
}

template <typename InType0, typename InType1, typename InType2,
          typename InType3, typename OutType0, typename OutType1,
          typename OutType2>
class ApolloRosMessageConverter<
    InputTypes<std::shared_ptr<InType0>, std::shared_ptr<InType1>,
               std::shared_ptr<InType2>, std::shared_ptr<InType3>>,
    OutputTypes<std::shared_ptr<OutType0>, std::shared_ptr<OutType1>,
                std::shared_ptr<OutType2>>> : public MessageConverter {
 public:
  ApolloRosMessageConverter() {}
  ~ApolloRosMessageConverter() override {}

  bool Init() override;

 protected:
  virtual bool ConvertMsg(
      InputTypes<std::shared_ptr<InType0>, std::shared_ptr<InType1>,
                 std::shared_ptr<InType2>, std::shared_ptr<InType3>>& input,
      OutputTypes<std::shared_ptr<OutType0>, std::shared_ptr<OutType1>,
                  std::shared_ptr<OutType2>>& output) = 0;
};

template <typename InType0, typename InType1, typename InType2,
          typename InType3, typename OutType0, typename OutType1,
          typename OutType2>
bool ApolloRosMessageConverter<
    InputTypes<std::shared_ptr<InType0>, std::shared_ptr<InType1>,
               std::shared_ptr<InType2>, std::shared_ptr<InType3>>,
    OutputTypes<std::shared_ptr<OutType0>, std::shared_ptr<OutType1>,
                std::shared_ptr<OutType2>>>::Init() {
  MessageConverter::Init();
  if (!init_.load()) {
    return false;
  }
  if (!LoadConfig(&converter_conf_)) {
    return false;
  }

  apollo::cyber::ReaderConfig reader_cfg_0;
  reader_cfg_0.channel_name = converter_conf_.apollo_channel_name_0();

  apollo::cyber::ReaderConfig reader_cfg_1;
  reader_cfg_1.channel_name = converter_conf_.apollo_channel_name_1();
  auto apollo_reader_1 =
      cyber_node_->template CreateReader<InType1>(reader_cfg_1);

  apollo::cyber::ReaderConfig reader_cfg_2;
  reader_cfg_2.channel_name = converter_conf_.apollo_channel_name_2();
  auto apollo_reader_2 =
      cyber_node_->template CreateReader<InType2>(reader_cfg_2);

  apollo::cyber::ReaderConfig reader_cfg_3;
  reader_cfg_3.channel_name = converter_conf_.apollo_channel_name_3();
  auto apollo_reader_3 =
      cyber_node_->template CreateReader<InType3>(reader_cfg_3);

  auto apollo_blocker_1 =
      blocker::BlockerManager::Instance()->GetBlocker<InType1>(
          reader_cfg_1.channel_name);
  auto apollo_blocker_2 =
      blocker::BlockerManager::Instance()->GetBlocker<InType2>(
          reader_cfg_2.channel_name);
  auto apollo_blocker_3 =
      blocker::BlockerManager::Instance()->GetBlocker<InType3>(
          reader_cfg_3.channel_name);

  std::string ros_topic_name_0 = converter_conf_.ros_topic_name_0();
  std::string ros_topic_name_1 = converter_conf_.ros_topic_name_1();
  std::string ros_topic_name_2 = converter_conf_.ros_topic_name_2();

#ifdef RCLCPP__RCLCPP_HPP_
  auto ros_publisher_0 =
      ros_node_->create_publisher<OutType0>(ros_topic_name_0, 10);
  auto ros_publisher_1 =
      ros_node_->create_publisher<OutType1>(ros_topic_name_1, 10);
  auto ros_publisher_2 =
      ros_node_->create_publisher<OutType2>(ros_topic_name_2, 10);
  ros_publishers_.push_back(std::move(ros_publisher_0));
  ros_publishers_.push_back(std::move(ros_publisher_1));
  ros_publishers_.push_back(std::move(ros_publisher_2));
  auto func = [this, apollo_blocker_1, apollo_blocker_2, apollo_blocker_3,
               ros_publisher_0, ros_publisher_1,
               ros_publisher_2](const std::shared_ptr<InType0> in) {
#else
  auto func = [this, apollo_blocker_1, apollo_blocker_2,
               apollo_blocker_3](const std::shared_ptr<InType0> in) {
#endif
    if (!apollo_blocker_1->IsPublishedEmpty() &&
        !apollo_blocker_1->IsPublishedEmpty() &&
        !apollo_blocker_1->IsPublishedEmpty()) {
      auto msg1 = apollo_blocker_1->GetLatestPublishedPtr();
      auto msg2 = apollo_blocker_2->GetLatestPublishedPtr();
      auto msg3 = apollo_blocker_3->GetLatestPublishedPtr();
      auto out_0 = std::make_shared<OutType0>();
      auto out_1 = std::make_shared<OutType1>();
      auto out_2 = std::make_shared<OutType2>();
      auto in_container =
          InputTypes<std::shared_ptr<InType0>, std::shared_ptr<InType1>,
                     std::shared_ptr<InType2>, std::shared_ptr<InType3>>{
              std::make_tuple(in, msg1, msg2, msg3)};
      auto out_container =
          OutputTypes<std::shared_ptr<OutType0>, std::shared_ptr<OutType1>,
                      std::shared_ptr<OutType2>>{
              std::make_tuple(out_0, out_1, out_2)};
      this->ConvertMsg(in_container, out_container);
#ifdef RCLCPP__RCLCPP_HPP_
      ros_publisher_0->publish(*out_0);
      ros_publisher_1->publish(*out_1);
      ros_publisher_2->publish(*out_2);
#endif
    }
  };
  auto apollo_reader_0 =
      cyber_node_->template CreateReader<InType0>(reader_cfg_0, func);
  apollo_readers_.push_back(std::move(apollo_reader_0));
  apollo_readers_.push_back(std::move(apollo_reader_1));
  apollo_readers_.push_back(std::move(apollo_reader_2));
  apollo_readers_.push_back(std::move(apollo_reader_3));

  return true;
}

template <typename InType0, typename InType1, typename InType2,
          typename InType3, typename OutType0, typename OutType1,
          typename OutType2, typename OutType3>
class ApolloRosMessageConverter<
    InputTypes<std::shared_ptr<InType0>, std::shared_ptr<InType1>,
               std::shared_ptr<InType2>, std::shared_ptr<InType3>>,
    OutputTypes<std::shared_ptr<OutType0>, std::shared_ptr<OutType1>,
                std::shared_ptr<OutType2>, std::shared_ptr<OutType3>>>
    : public MessageConverter {
 public:
  ApolloRosMessageConverter() {}
  ~ApolloRosMessageConverter() override {}

  bool Init() override;

 protected:
  virtual bool ConvertMsg(
      InputTypes<std::shared_ptr<InType0>, std::shared_ptr<InType1>,
                 std::shared_ptr<InType2>, std::shared_ptr<InType3>>& input,
      OutputTypes<std::shared_ptr<OutType0>, std::shared_ptr<OutType1>,
                  std::shared_ptr<OutType2>, std::shared_ptr<OutType3>>&
          output) = 0;
};

template <typename InType0, typename InType1, typename InType2,
          typename InType3, typename OutType0, typename OutType1,
          typename OutType2, typename OutType3>
bool ApolloRosMessageConverter<
    InputTypes<std::shared_ptr<InType0>, std::shared_ptr<InType1>,
               std::shared_ptr<InType2>, std::shared_ptr<InType3>>,
    OutputTypes<std::shared_ptr<OutType0>, std::shared_ptr<OutType1>,
                std::shared_ptr<OutType2>, std::shared_ptr<OutType3>>>::Init() {
  MessageConverter::Init();
  if (!init_.load()) {
    return false;
  }
  if (!LoadConfig(&converter_conf_)) {
    return false;
  }

  apollo::cyber::ReaderConfig reader_cfg_0;
  reader_cfg_0.channel_name = converter_conf_.apollo_channel_name_0();

  apollo::cyber::ReaderConfig reader_cfg_1;
  reader_cfg_1.channel_name = converter_conf_.apollo_channel_name_1();
  auto apollo_reader_1 =
      cyber_node_->template CreateReader<InType1>(reader_cfg_1);

  apollo::cyber::ReaderConfig reader_cfg_2;
  reader_cfg_2.channel_name = converter_conf_.apollo_channel_name_2();
  auto apollo_reader_2 =
      cyber_node_->template CreateReader<InType2>(reader_cfg_2);

  apollo::cyber::ReaderConfig reader_cfg_3;
  reader_cfg_3.channel_name = converter_conf_.apollo_channel_name_3();
  auto apollo_reader_3 =
      cyber_node_->template CreateReader<InType3>(reader_cfg_3);

  auto apollo_blocker_1 =
      blocker::BlockerManager::Instance()->GetBlocker<InType1>(
          reader_cfg_1.channel_name);
  auto apollo_blocker_2 =
      blocker::BlockerManager::Instance()->GetBlocker<InType2>(
          reader_cfg_2.channel_name);
  auto apollo_blocker_3 =
      blocker::BlockerManager::Instance()->GetBlocker<InType3>(
          reader_cfg_3.channel_name);

  std::string ros_topic_name_0 = converter_conf_.ros_topic_name_0();
  std::string ros_topic_name_1 = converter_conf_.ros_topic_name_1();
  std::string ros_topic_name_2 = converter_conf_.ros_topic_name_2();
  std::string ros_topic_name_3 = converter_conf_.ros_topic_name_3();

#ifdef RCLCPP__RCLCPP_HPP_
  auto ros_publisher_0 =
      ros_node_->create_publisher<OutType0>(ros_topic_name_0, 10);
  auto ros_publisher_1 =
      ros_node_->create_publisher<OutType1>(ros_topic_name_1, 10);
  auto ros_publisher_2 =
      ros_node_->create_publisher<OutType2>(ros_topic_name_2, 10);
  auto ros_publisher_3 =
      ros_node_->create_publisher<OutType2>(ros_topic_name_3, 10);
  ros_publishers_.push_back(std::move(ros_publisher_0));
  ros_publishers_.push_back(std::move(ros_publisher_1));
  ros_publishers_.push_back(std::move(ros_publisher_2));
  ros_publishers_.push_back(std::move(ros_publisher_3));
  auto func = [this, apollo_blocker_1, apollo_blocker_2, apollo_blocker_3,
               ros_publisher_0, ros_publisher_1, ros_publisher_2,
               ros_publisher_3](const std::shared_ptr<InType0> in) {
#else
  auto func = [this, apollo_blocker_1, apollo_blocker_2,
               apollo_blocker_3](const std::shared_ptr<InType0> in) {
#endif
    if (!apollo_blocker_1->IsPublishedEmpty() &&
        !apollo_blocker_2->IsPublishedEmpty() &&
        !apollo_blocker_3->IsPublishedEmpty()) {
      auto msg1 = apollo_blocker_1->GetLatestPublishedPtr();
      auto msg2 = apollo_blocker_2->GetLatestPublishedPtr();
      auto msg3 = apollo_blocker_3->GetLatestPublishedPtr();
      auto out_0 = std::make_shared<OutType0>();
      auto out_1 = std::make_shared<OutType1>();
      auto out_2 = std::make_shared<OutType2>();
      auto out_3 = std::make_shared<OutType3>();
      auto in_container =
          InputTypes<std::shared_ptr<InType0>, std::shared_ptr<InType1>,
                     std::shared_ptr<InType2>, std::shared_ptr<InType3>>{
              std::make_tuple(in, msg1, msg2, msg3)};
      auto out_container =
          OutputTypes<std::shared_ptr<OutType0>, std::shared_ptr<OutType1>,
                      std::shared_ptr<OutType2>, std::shared_ptr<OutType3>>{
              std::make_tuple(out_0, out_1, out_2, out_3)};
      this->ConvertMsg(in_container, out_container);
#ifdef RCLCPP__RCLCPP_HPP_
      ros_publisher_0->publish(*out_0);
      ros_publisher_1->publish(*out_1);
      ros_publisher_2->publish(*out_2);
      ros_publisher_3->publish(*out_2);
#endif
    }
  };
  auto apollo_reader_0 =
      cyber_node_->template CreateReader<InType0>(reader_cfg_0, func);
  apollo_readers_.push_back(std::move(apollo_reader_0));
  apollo_readers_.push_back(std::move(apollo_reader_1));
  apollo_readers_.push_back(std::move(apollo_reader_2));
  apollo_readers_.push_back(std::move(apollo_reader_3));

  return true;
}

}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_APOLLO_ROS_MESSAGE_CONVERTER_H_
