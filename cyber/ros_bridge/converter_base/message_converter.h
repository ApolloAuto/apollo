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

#ifndef CYBER_MESSAGE_CONVERTER_H_
#define CYBER_MESSAGE_CONVERTER_H_

#include <atomic>   // NOLINT
#include <memory>   // NOLINT
#include <string>   // NOLINT
#include <thread>   // NOLINT
#include <utility>  // NOLINT
#include <vector>   // NOLINT

#include <cxxabi.h>  // NOLINT

#include "cyber/ros_bridge/proto/converter_conf.pb.h"

#include "cyber/cyber.h"
#include "cyber/node/reader_base.h"
#include "cyber/node/writer_base.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "cyber/ros_bridge/common/macros.h"

#if __has_include("rclcpp/rclcpp.hpp")
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "ros_adapter/ros_distro.h"
#endif

namespace apollo {
namespace cyber {

class MessageConverter {
 public:
  MessageConverter() : init_(false) {}

  virtual ~MessageConverter() {}

  virtual bool Init() {
    if (init_.exchange(true)) {
      return true;
    }
    LoadConfig(&converter_conf_);
    cyber_node_ = std::move(
        CreateNode(node_name_ + "_" + converter_conf_.name() + "_apollo"));
#ifdef RCLCPP__RCLCPP_HPP_
    ros_node_ = std::make_shared<::rclcpp::Node>(
        node_name_ + "_" + converter_conf_.name() + "_ros");
    ros_node_exec_ =
        std::make_shared<::rclcpp::executors::SingleThreadedExecutor>();
#endif
    return true;
  }

#ifdef RCLCPP__RCLCPP_HPP_
  void NodeSpin() {
    ros_node_exec_->add_node(std::shared_ptr<rclcpp::Node>(ros_node_));
    ros_node_exec_->spin();
  }
#endif

  bool IsInit() const { return init_.load(); }

 protected:
  bool LoadConfig(ConverterConf* config) {
    int status;
    std::string class_name =
        abi::__cxa_demangle(typeid(*this).name(), 0, 0, &status);
    std::string delimiter = "::";
    std::string sub_class_name;
    std::string conf_file_prefix;

    auto pos = class_name.rfind(delimiter);
    if (pos == std::string::npos) {
      sub_class_name = class_name;
    } else {
      sub_class_name = class_name.substr(pos + delimiter.length());
    }

    for (int i = 0; i < sub_class_name.length(); i++) {
      if (std::isupper(sub_class_name[i]) && i > 0) {
        conf_file_prefix.push_back('_');
      }
      conf_file_prefix.push_back(std::tolower(sub_class_name[i]));
    }

    std::string config_path =
        apollo::cyber::plugin_manager::PluginManager::Instance()
            ->GetPluginConfPath<MessageConverter>(
                class_name, "conf/" + conf_file_prefix + ".pb.txt");
    if (!apollo::cyber::common::PathExists(config_path)) {
      config_path = apollo::cyber::plugin_manager::PluginManager::Instance()
                        ->GetPluginConfPath<MessageConverter>(
                            class_name, std::string("conf/default.pb.txt"));
    }

    if (!apollo::cyber::common::GetProtoFromFile(config_path, config)) {
      AERROR << "Load config of " << class_name << " failed!";
      return false;
    }
    AINFO << "Load the [" << class_name
          << "] config file successfully, file path: " << config_path;
    return true;
  }

 protected:
  std::atomic<bool> init_;
  std::unique_ptr<apollo::cyber::Node> cyber_node_;
  std::vector<std::shared_ptr<apollo::cyber::proto::RoleAttributes>>
      apollo_attrs_;
  std::vector<std::shared_ptr<apollo::cyber::ReaderBase>> apollo_readers_;
  std::vector<std::shared_ptr<apollo::cyber::WriterBase>> apollo_writers_;
#ifdef RCLCPP__RCLCPP_HPP_
  std::vector<std::shared_ptr<::rclcpp::PublisherBase>> ros_publishers_;
  std::vector<std::shared_ptr<::rclcpp::SubscriptionBase>> ros_subscriptions_;
#if defined(ROS_DISTRO_FOXY) || defined(ROS_DISTRO_GALACTIC)
  std::vector<std::shared_ptr<::message_filters::SubscriberBase>>
#else
  std::vector<std::shared_ptr<::message_filters::SubscriberBase<rclcpp::Node>>>
#endif
      ros_msg_subs_;
  std::shared_ptr<::rclcpp::Node> ros_node_ = nullptr;
  std::shared_ptr<::rclcpp::executors::SingleThreadedExecutor> ros_node_exec_ =
      nullptr;
  std::shared_ptr<std::thread> ros_spin_thread_;
#endif
  const std::string node_name_ = "converter_base";
  ConverterConf converter_conf_;
};

}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_MESSAGE_CONVERTER_H_
