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

#include "modules/routing/routing_component.h"
// C++标准库，通常用来支持 std::move, std::pair 等工具
#include <utility>
// 包含了与 adapter 和 routing 相关的 gflags 参数定义
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/routing/common/routing_gflags.h"

// 告诉编译器，这个参数变量存在，可以在当前文件中使用
DECLARE_string(flagfile);

namespace apollo {
namespace routing {

bool RoutingComponent::Init() {
  // 创建一个空的路由配置 protobuf 对象
  RoutingConfig routing_conf;

  // 从组件配置文件（通常是 protobuf 格式的 .conf 文件）中读取配置，填充到 routing_conf
  ACHECK(cyber::ComponentBase::GetProtoConfig(&routing_conf))
      << "Unable to load routing conf file: "
      << cyber::ComponentBase::ConfigFilePath();

  AINFO << "Config file: " << cyber::ComponentBase::ConfigFilePath()
        << " is loaded.";
  
  // 创建一个 Cyber RT 通信的写者 response_writer_，用于发布路由规划结果
  apollo::cyber::proto::RoleAttributes attr;
  attr.set_channel_name(routing_conf.topic_config().routing_response_topic());
  auto qos = attr.mutable_qos_profile();
  qos->set_history(apollo::cyber::proto::QosHistoryPolicy::HISTORY_KEEP_LAST);
  qos->set_reliability(
      apollo::cyber::proto::QosReliabilityPolicy::RELIABILITY_RELIABLE);
  qos->set_durability(
      apollo::cyber::proto::QosDurabilityPolicy::DURABILITY_TRANSIENT_LOCAL);
  // 用 node_->CreateWriter<RoutingResponse>(attr) 创建写者
  response_writer_ = node_->CreateWriter<routing::RoutingResponse>(attr);

// 创建另一个写者 response_history_writer_，发布历史的路由响应消息，话题为 routing_response_history_topic
  apollo::cyber::proto::RoleAttributes attr_history;
  attr_history.set_channel_name(
      routing_conf.topic_config().routing_response_history_topic());
  auto qos_history = attr_history.mutable_qos_profile();
  qos_history->set_history(
      apollo::cyber::proto::QosHistoryPolicy::HISTORY_KEEP_LAST);
  qos_history->set_reliability(
      apollo::cyber::proto::QosReliabilityPolicy::RELIABILITY_RELIABLE);
  qos_history->set_durability(
      apollo::cyber::proto::QosDurabilityPolicy::DURABILITY_TRANSIENT_LOCAL);
  response_history_writer_ =
      node_->CreateWriter<routing::RoutingResponse>(attr_history);

  // 获取当前组件的 weak_ptr 智能指针，避免 lambda 捕获时产生循环引用导致内存泄漏
  std::weak_ptr<RoutingComponent> self =
      std::dynamic_pointer_cast<RoutingComponent>(shared_from_this());

  timer_.reset(new ::apollo::cyber::Timer(
      FLAGS_routing_response_history_interval_ms,  // 1000 ms
      [self, this]() {
        auto ptr = self.lock();
        if (ptr) {
          std::lock_guard<std::mutex> guard(this->mutex_);
          if (this->response_ != nullptr) {
            auto timestamp = apollo::cyber::Clock::NowInSeconds();
            response_->mutable_header()->set_timestamp_sec(timestamp);
            this->response_history_writer_->Write(*response_);
          }
        }
      },
      false));
  timer_->Start();
  
  // routing_（内部的路由核心算法实例）的初始化和启动函数
  return routing_.Init().ok() && routing_.Start().ok();
}

bool RoutingComponent::Proc(
    const std::shared_ptr<routing::RoutingRequest>& request) {
  // 新建一个空的 RoutingResponse 用于存放结果
  auto response = std::make_shared<routing::RoutingResponse>();
  // routing_ 是一个成员变量，类型是 Routing，真正执行路径规划逻辑
  if (!routing_.Process(request, response.get())) {
    return false;
  }

  // 给响应加上通用头部信息（时间戳、模块名等）
  common::util::FillHeader(node_->Name(), response.get());

  // 发布路由结果
  response_writer_->Write(response);

  // 存储最新响应（用于定时器回放历史）
  {
    std::lock_guard<std::mutex> guard(mutex_);
    response_ = std::move(response);
  }
  return true;
}

}  // namespace routing
}  // namespace apollo
