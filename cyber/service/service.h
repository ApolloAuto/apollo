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

#ifndef CYBER_SERVICE_SERVICE_H_
#define CYBER_SERVICE_SERVICE_H_

#include <memory>
#include <string>

#include "cyber/common/types.h"
#include "cyber/node/node_channel_impl.h"
#include "cyber/service/service_base.h"
#include "cyber/scheduler/scheduler.h"

namespace apollo {
namespace cyber {

using cyber::scheduler::Scheduler;

template <typename Request, typename Response>
class Service : public ServiceBase {
 public:
  using ServiceCallback = std::function<void(const std::shared_ptr<Request>&,
                                             std::shared_ptr<Response>&)>;
  Service(const std::string& node_name, const std::string& service_name,
          const ServiceCallback& service_callback)
      : ServiceBase(service_name),
        node_name_(node_name),
        init_(false),
        service_callback_(service_callback),
        request_channel_(service_name + SRV_CHANNEL_REQ_SUFFIX),
        response_channel_(service_name + SRV_CHANNEL_RES_SUFFIX) {}

  Service(const std::string& node_name, const std::string& service_name,
          ServiceCallback&& service_callback)
      : ServiceBase(service_name),
        node_name_(node_name),
        init_(false),
        service_callback_(service_callback),
        request_channel_(service_name + SRV_CHANNEL_REQ_SUFFIX),
        response_channel_(service_name + SRV_CHANNEL_RES_SUFFIX) {}

  Service() = delete;
  ~Service() { init_ = false; }
  bool Init();
  void destroy();

 private:
  void HandleRequest(const std::shared_ptr<Request>& request,
                     const transport::MessageInfo& message_info);

  void SendResponse(const transport::MessageInfo& message_info,
                    const std::shared_ptr<Response>& response);
  std::string node_name_;
  bool init_;
  ServiceCallback service_callback_;

  std::function<void(const std::shared_ptr<Request>&,
                     const transport::MessageInfo&)>
      request_callback_;
  std::shared_ptr<transport::Transmitter<Response>> response_transmitter_;
  std::shared_ptr<transport::Receiver<Request>> request_receiver_;
  std::string request_channel_;
  std::string response_channel_;
  std::mutex service_handle_request_mutex_;
};

template <typename Request, typename Response>
void Service<Request, Response>::destroy() {}

template <typename Request, typename Response>
bool Service<Request, Response>::Init() {
  if (init_) {
    return true;
  }
  proto::RoleAttributes role;
  role.set_node_name(node_name_);
  role.set_channel_name(response_channel_);
  auto channel_id = common::GlobalData::RegisterChannel(response_channel_);
  role.set_channel_id(channel_id);
  role.mutable_qos_profile()->CopyFrom(
      transport::QosProfileConf::QOS_PROFILE_SERVICES_DEFAULT);
  auto transport = transport::Transport::Instance();
  response_transmitter_ =
      transport->CreateTransmitter<Response>(role, proto::OptionalMode::RTPS);
  if (response_transmitter_ == nullptr) {
    AERROR << " Create response pub failed.";
    return false;
  }

  request_callback_ =
      std::bind(&Service<Request, Response>::HandleRequest, this,
                std::placeholders::_1, std::placeholders::_2);

  role.set_channel_name(request_channel_);
  channel_id = common::GlobalData::RegisterChannel(request_channel_);
  role.set_channel_id(channel_id);
  request_receiver_ = transport->CreateReceiver<Request>(
      role,
      [=](const std::shared_ptr<Request>& request,
          const transport::MessageInfo& message_info,
          const proto::RoleAttributes& reader_attr) {
        (void)reader_attr;
        std::thread _thread =
            std::thread(request_callback_, request, message_info);
        Scheduler::Instance()->SetInnerThreadAttr(&_thread, "service");
        _thread.detach();
      },
      proto::OptionalMode::RTPS);
  if (request_receiver_ == nullptr) {
    AERROR << " Create request sub failed." << request_channel_;
    return false;
  }
  init_ = true;
  return true;
}

template <typename Request, typename Response>
void Service<Request, Response>::HandleRequest(
    const std::shared_ptr<Request>& request,
    const transport::MessageInfo& message_info) {
  if (!init_) {
    // LOG_DEBUG << "not inited error.";
    return;
  }
  ADEBUG << "handling request:" << request_channel_;
  std::lock_guard<std::mutex> lk(service_handle_request_mutex_);
  auto response = std::make_shared<Response>();
  service_callback_(request, response);
  transport::MessageInfo msg_info(message_info);
  msg_info.set_sender_id(response_transmitter_->id());
  SendResponse(msg_info, response);
}

template <typename Request, typename Response>
void Service<Request, Response>::SendResponse(
    const transport::MessageInfo& message_info,
    const std::shared_ptr<Response>& response) {
  if (!init_) {
    // LOG_DEBUG << "not inited error.";
    return;
  }
  // publish return value ?
  // LOG_DEBUG << "send response id:" << message_id.sequence_number;
  response_transmitter_->Transmit(response, message_info);
}

}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_SERVICE_SERVICE_H_
