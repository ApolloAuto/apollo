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

#ifndef CYBER_ROS_APOLLO_SINGLE_H_
#define CYBER_ROS_APOLLO_SINGLE_H_

#include <memory>
#include <utility>
#include "cyber/cyber.h"
#include "cyber/ros_bridge/converter_base/converter_base.h"

namespace apollo {
namespace cyber {

template <typename InType0, typename OutType0>
class RosApolloMessageConverter<InputTypes<std::shared_ptr<InType0>>,
                                OutputTypes<std::shared_ptr<OutType0>>>
    : public MessageConverter {
 private:
  std::shared_ptr<Writer<OutType0>> apollo_writer_0_ = nullptr;

 public:
  RosApolloMessageConverter() {}
  ~RosApolloMessageConverter() override {}

  bool Init() override {
    MessageConverter::Init();
    if (!init_.load()) {
      return false;
    }
    if (!LoadConfig(&converter_conf_)) {
      AERROR << "load config failed";
      return false;
    }

    auto writer_attrs =
        std::make_shared<apollo::cyber::proto::RoleAttributes>();
    writer_attrs->set_channel_name(converter_conf_.apollo_channel_name_0());
    apollo_attrs_.push_back(writer_attrs);
    auto ros_topic_name_0 = converter_conf_.ros_topic_name_0();

    apollo_writer_0_ =
        cyber_node_->template CreateWriter<OutType0>(*writer_attrs);
#ifdef RCLCPP__RCLCPP_HPP_
    ros_subscriptions_.push_back(
        std::move(ros_node_->create_subscription<InType0>(
            ros_topic_name_0, 10,
            std::bind(
                &RosApolloMessageConverter<
                    InputTypes<std::shared_ptr<InType0>>,
                    OutputTypes<std::shared_ptr<OutType0>>>::TopicCallback,
                this, std::placeholders::_1))));
    ros_spin_thread_ =
        std::make_shared<std::thread>(&MessageConverter::NodeSpin, this);
#endif
    return true;
  }

 protected:
  virtual bool ConvertMsg(InputTypes<std::shared_ptr<InType0>>& input,
                          OutputTypes<std::shared_ptr<OutType0>>& output) = 0;

#ifdef RCLCPP__RCLCPP_HPP_
 private:
  void TopicCallback(std::shared_ptr<InType0> ros_msg0) {
    auto out = std::make_shared<OutType0>();
    typename InType0::SharedPtr internal_in_prt =
        std::make_shared<InType0>(*ros_msg0.get());
    auto in_container =
        InputTypes<std::shared_ptr<InType0>>{std::make_tuple(internal_in_prt)};
    auto out_container =
        OutputTypes<std::shared_ptr<OutType0>>{std::make_tuple(out)};
    this->ConvertMsg(in_container, out_container);
    apollo_writer_0_->Write(out);
  }
#endif
};

template <typename InType0, typename OutType0, typename OutType1>
class RosApolloMessageConverter<
    InputTypes<std::shared_ptr<InType0>>,
    OutputTypes<std::shared_ptr<OutType0>, std::shared_ptr<OutType1>>>
    : public MessageConverter {
 private:
  std::shared_ptr<Writer<OutType0>> apollo_writer_0_ = nullptr;
  std::shared_ptr<Writer<OutType1>> apollo_writer_1_ = nullptr;

 public:
  RosApolloMessageConverter() {}
  ~RosApolloMessageConverter() override {}

  bool Init() override {
    MessageConverter::Init();
    if (!init_.load()) {
      return false;
    }
    if (!LoadConfig(&converter_conf_)) {
      return false;
    }

    apollo_attrs_.push_back(
        std::make_shared<apollo::cyber::proto::RoleAttributes>());
    apollo_attrs_[0]->set_channel_name(converter_conf_.apollo_channel_name_0());

    apollo_attrs_.push_back(
        std::make_shared<apollo::cyber::proto::RoleAttributes>());
    apollo_attrs_[1]->set_channel_name(converter_conf_.apollo_channel_name_1());

    auto ros_topic_name_0 = converter_conf_.ros_topic_name_0();

    apollo_writer_0_ =
        cyber_node_->template CreateWriter<OutType0>(*apollo_attrs_[0]);
    apollo_writer_1_ =
        cyber_node_->template CreateWriter<OutType1>(*apollo_attrs_[1]);
#ifdef RCLCPP__RCLCPP_HPP_
    ros_subscriptions_.push_back(
        std::move(ros_node_->create_subscription<InType0>(
            ros_topic_name_0, 10,
            std::bind(
                &RosApolloMessageConverter<
                    InputTypes<std::shared_ptr<InType0>>,
                    OutputTypes<std::shared_ptr<OutType0>,
                                std::shared_ptr<OutType1>>>::TopicCallback,
                this, std::placeholders::_1))));
    ros_spin_thread_ =
        std::make_shared<std::thread>(&MessageConverter::NodeSpin, this);
#endif
    return true;
  }

 protected:
  virtual bool ConvertMsg(InputTypes<std::shared_ptr<InType0>>& input,
                          OutputTypes<std::shared_ptr<OutType0>,
                                      std::shared_ptr<OutType1>>& output) = 0;

#ifdef RCLCPP__RCLCPP_HPP_
 private:
  void TopicCallback(std::shared_ptr<InType0> ros_msg0) {
    auto out_0 = std::make_shared<OutType0>();
    auto out_1 = std::make_shared<OutType1>();
    typename InType0::SharedPtr internal_in_prt =
        std::make_shared<InType0>(*ros_msg0.get());
    auto in_container =
        InputTypes<std::shared_ptr<InType0>>{std::make_tuple(internal_in_prt)};
    auto out_container =
        OutputTypes<std::shared_ptr<OutType0>, std::shared_ptr<OutType1>>{
            std::make_tuple(out_0, out_1)};
    this->ConvertMsg(in_container, out_container);
    apollo_writer_0_->Write(out_0);
    apollo_writer_1_->Write(out_1);
  }
#endif
};

template <typename InType0, typename OutType0, typename OutType1,
          typename OutType2>
class RosApolloMessageConverter<
    InputTypes<std::shared_ptr<InType0>>,
    OutputTypes<std::shared_ptr<OutType0>, std::shared_ptr<OutType1>,
                std::shared_ptr<OutType2>>> : public MessageConverter {
 private:
  std::shared_ptr<Writer<OutType0>> apollo_writer_0_ = nullptr;
  std::shared_ptr<Writer<OutType1>> apollo_writer_1_ = nullptr;
  std::shared_ptr<Writer<OutType2>> apollo_writer_2_ = nullptr;

 public:
  RosApolloMessageConverter() {}
  ~RosApolloMessageConverter() override {}

  bool Init() override {
    MessageConverter::Init();
    if (!init_.load()) {
      return false;
    }
    if (!LoadConfig(&converter_conf_)) {
      return false;
    }

    apollo_attrs_.push_back(
        std::make_shared<apollo::cyber::proto::RoleAttributes>());
    apollo_attrs_[0]->set_channel_name(converter_conf_.apollo_channel_name_0());

    apollo_attrs_.push_back(
        std::make_shared<apollo::cyber::proto::RoleAttributes>());
    apollo_attrs_[1]->set_channel_name(converter_conf_.apollo_channel_name_1());

    apollo_attrs_.push_back(
        std::make_shared<apollo::cyber::proto::RoleAttributes>());
    apollo_attrs_[2]->set_channel_name(converter_conf_.apollo_channel_name_2());

    auto ros_topic_name_0 = converter_conf_.ros_topic_name_0();

    apollo_writer_0_ =
        cyber_node_->template CreateWriter<OutType0>(*apollo_attrs_[0]);
    apollo_writer_1_ =
        cyber_node_->template CreateWriter<OutType1>(*apollo_attrs_[1]);
    apollo_writer_2_ =
        cyber_node_->template CreateWriter<OutType2>(*apollo_attrs_[2]);
#ifdef RCLCPP__RCLCPP_HPP_
    ros_subscriptions_.push_back(
        std::move(ros_node_->create_subscription<InType0>(
            ros_topic_name_0, 10,
            std::bind(
                &RosApolloMessageConverter<
                    InputTypes<std::shared_ptr<InType0>>,
                    OutputTypes<std::shared_ptr<OutType0>,
                                std::shared_ptr<OutType1>,
                                std::shared_ptr<OutType2>>>::TopicCallback,
                this, std::placeholders::_1))));
    ros_spin_thread_ =
        std::make_shared<std::thread>(&MessageConverter::NodeSpin, this);
#endif
    return true;
  }

 protected:
  virtual bool ConvertMsg(
      InputTypes<std::shared_ptr<InType0>>& input,
      OutputTypes<std::shared_ptr<OutType0>, std::shared_ptr<OutType1>,
                  std::shared_ptr<OutType2>>& output) = 0;

#ifdef RCLCPP__RCLCPP_HPP_
 private:
  void TopicCallback(std::shared_ptr<InType0> ros_msg0) {
    auto out_0 = std::make_shared<OutType0>();
    auto out_1 = std::make_shared<OutType1>();
    auto out_2 = std::make_shared<OutType2>();
    typename InType0::SharedPtr internal_in_prt =
        std::make_shared<InType0>(*ros_msg0.get());
    auto in_container =
        InputTypes<std::shared_ptr<InType0>>{std::make_tuple(internal_in_prt)};
    auto out_container =
        OutputTypes<std::shared_ptr<OutType0>, std::shared_ptr<OutType1>,
                    std::shared_ptr<OutType2>>{
            std::make_tuple(out_0, out_1, out_2)};
    this->ConvertMsg(in_container, out_container);
    apollo_writer_0_->Write(out_0);
    apollo_writer_1_->Write(out_1);
    apollo_writer_2_->Write(out_2);
  }
#endif
};

template <typename InType0, typename OutType0, typename OutType1,
          typename OutType2, typename OutType3>
class RosApolloMessageConverter<
    InputTypes<std::shared_ptr<InType0>>,
    OutputTypes<std::shared_ptr<OutType0>, std::shared_ptr<OutType1>,
                std::shared_ptr<OutType2>, std::shared_ptr<OutType3>>>
    : public MessageConverter {
 private:
  std::shared_ptr<Writer<OutType0>> apollo_writer_0_ = nullptr;
  std::shared_ptr<Writer<OutType1>> apollo_writer_1_ = nullptr;
  std::shared_ptr<Writer<OutType2>> apollo_writer_2_ = nullptr;
  std::shared_ptr<Writer<OutType3>> apollo_writer_3_ = nullptr;

 public:
  RosApolloMessageConverter() {}
  ~RosApolloMessageConverter() override {}

  bool Init() override {
    MessageConverter::Init();
    if (!init_.load()) {
      return false;
    }
    if (!LoadConfig(&converter_conf_)) {
      return false;
    }

    apollo_attrs_.push_back(
        std::make_shared<apollo::cyber::proto::RoleAttributes>());
    apollo_attrs_[0]->set_channel_name(converter_conf_.apollo_channel_name_0());

    apollo_attrs_.push_back(
        std::make_shared<apollo::cyber::proto::RoleAttributes>());
    apollo_attrs_[1]->set_channel_name(converter_conf_.apollo_channel_name_1());

    apollo_attrs_.push_back(
        std::make_shared<apollo::cyber::proto::RoleAttributes>());
    apollo_attrs_[2]->set_channel_name(converter_conf_.apollo_channel_name_2());

    apollo_attrs_.push_back(
        std::make_shared<apollo::cyber::proto::RoleAttributes>());
    apollo_attrs_[3]->set_channel_name(converter_conf_.apollo_channel_name_3());

    auto ros_topic_name_0 = converter_conf_.ros_topic_name_0();

    apollo_writer_0_ =
        cyber_node_->template CreateWriter<OutType0>(*apollo_attrs_[0]);
    apollo_writer_1_ =
        cyber_node_->template CreateWriter<OutType1>(*apollo_attrs_[1]);
    apollo_writer_2_ =
        cyber_node_->template CreateWriter<OutType2>(*apollo_attrs_[2]);
    apollo_writer_3_ =
        cyber_node_->template CreateWriter<OutType3>(*apollo_attrs_[3]);
#ifdef RCLCPP__RCLCPP_HPP_
    ros_subscriptions_.push_back(
        std::move(ros_node_->create_subscription<InType0>(
            ros_topic_name_0, 10,
            std::bind(
                &RosApolloMessageConverter<
                    InputTypes<std::shared_ptr<InType0>>,
                    OutputTypes<std::shared_ptr<OutType0>,
                                std::shared_ptr<OutType1>,
                                std::shared_ptr<OutType2>,
                                std::shared_ptr<OutType3>>>::TopicCallback,
                this, std::placeholders::_1))));
    ros_spin_thread_ =
        std::make_shared<std::thread>(&MessageConverter::NodeSpin, this);
#endif
    return true;
  }

 protected:
  virtual bool ConvertMsg(
      InputTypes<std::shared_ptr<InType0>>& input,
      OutputTypes<std::shared_ptr<OutType0>, std::shared_ptr<OutType1>,
                  std::shared_ptr<OutType2>, std::shared_ptr<OutType3>>&
          output) = 0;

#ifdef RCLCPP__RCLCPP_HPP_
 private:
  void TopicCallback(std::shared_ptr<InType0> ros_msg0) {
    auto out_0 = std::make_shared<OutType0>();
    auto out_1 = std::make_shared<OutType1>();
    auto out_2 = std::make_shared<OutType2>();
    auto out_3 = std::make_shared<OutType3>();
    typename InType0::SharedPtr internal_in_prt =
        std::make_shared<InType0>(*ros_msg0.get());
    auto in_container =
        InputTypes<std::shared_ptr<InType0>>{std::make_tuple(internal_in_prt)};
    auto out_container =
        OutputTypes<std::shared_ptr<OutType0>, std::shared_ptr<OutType1>,
                    std::shared_ptr<OutType2>, std::shared_ptr<OutType3>>{
            std::make_tuple(out_0, out_1, out_2, out_3)};
    this->ConvertMsg(in_container, out_container);
    apollo_writer_0_->Write(out_0);
    apollo_writer_1_->Write(out_1);
    apollo_writer_2_->Write(out_2);
    apollo_writer_3_->Write(out_3);
  }
#endif
};

}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_APOLLO_ROS_MESSAGE_CONVERTER_H_
