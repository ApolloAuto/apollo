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

#ifndef PYTHON_WRAPPER_PY_NODE_H_
#define PYTHON_WRAPPER_PY_NODE_H_

#include <unistd.h>

#include <deque>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include "cyber/cyber.h"
#include "cyber/init.h"
#include "cyber/message/protobuf_factory.h"
#include "cyber/message/py_message.h"
#include "cyber/message/raw_message.h"
#include "cyber/node/node.h"
#include "cyber/node/reader.h"
#include "cyber/node/writer.h"

namespace apollo {
namespace cyber {

class PyWriter {
 public:
  PyWriter(const std::string &channel, const std::string &type,
           const uint32_t qos_depth, apollo::cyber::Node *node)
      : channel_name_(channel),
        data_type_(type),
        qos_depth_(qos_depth),
        node_(node) {
    std::string proto_desc("");
    message::ProtobufFactory::Instance()->GetDescriptorString(type,
                                                              &proto_desc);
    if (proto_desc.empty()) {
      AWARN << "cpp cann't find proto_desc msgtyp->" << data_type_;
      return;
    }
    proto::RoleAttributes role_attr;
    role_attr.set_channel_name(channel_name_);
    role_attr.set_message_type(data_type_);
    role_attr.set_proto_desc(proto_desc);
    auto qos_profile = role_attr.mutable_qos_profile();
    qos_profile->set_depth(qos_depth_);
    writer_ =
        node_->CreateWriter<apollo::cyber::message::PyMessageWrap>(role_attr);
  }

  ~PyWriter() {}

  int write(const std::string &data) {
    auto message =
        std::make_shared<cyber::message::PyMessageWrap>(data, data_type_);
    message->set_type_name(data_type_);
    return writer_->Write(message);
  }

 private:
  std::string channel_name_;
  std::string data_type_;
  uint32_t qos_depth_;
  apollo::cyber::Node *node_ = nullptr;
  std::shared_ptr<apollo::cyber::Writer<apollo::cyber::message::PyMessageWrap>>
      writer_;
};

class PyReader {
 public:
  PyReader(const std::string &channel, const std::string &type,
           apollo::cyber::Node *node)
      : node_(node), channel_name_(channel), data_type_(type), func_(nullptr) {
    auto f = [this](
        const std::shared_ptr<const apollo::cyber::message::PyMessageWrap>
            &request) { this->cb(request); };
    reader_ =
        node_->CreateReader<apollo::cyber::message::PyMessageWrap>(channel, f);
  }

  ~PyReader() {}

  void register_func(int (*func)(const char *)) { func_ = func; }

  std::string read(bool wait = false) {
    std::string msg("");
    std::unique_lock<std::mutex> ul(msg_lock_);
    if (!cache_.empty()) {
      msg = std::move(cache_.front());
      cache_.pop_front();
    }

    if (!wait) {
      return msg;
    }

    msg_cond_.wait(ul, [this] { return !this->cache_.empty(); });
    if (!cache_.empty()) {
      msg = std::move(cache_.front());
      cache_.pop_front();
    }

    return msg;
  }

 private:
  void cb(const std::shared_ptr<const apollo::cyber::message::PyMessageWrap>
              &message) {
    {
      std::lock_guard<std::mutex> lg(msg_lock_);
      cache_.push_back(message->data());
    }
    if (func_) {
      func_(channel_name_.c_str());
    }
    msg_cond_.notify_one();
  }

  apollo::cyber::Node *node_;
  std::string channel_name_;
  std::string data_type_;
  int (*func_)(const char *) = nullptr;
  std::shared_ptr<apollo::cyber::Reader<apollo::cyber::message::PyMessageWrap>>
      reader_;
  std::deque<std::string> cache_;
  std::mutex msg_lock_;
  std::condition_variable msg_cond_;
};

class PyService {
 public:
  PyService(const std::string &service_name, const std::string &data_type,
            apollo::cyber::Node *node)
      : node_(node),
        service_name_(service_name),
        data_type_(data_type),
        func_(nullptr) {
    auto f = [this](
        const std::shared_ptr<const apollo::cyber::message::PyMessageWrap>
            &request,
        std::shared_ptr<apollo::cyber::message::PyMessageWrap> &response) {
      this->cb(request, response);
    };
    service_ = node_->CreateService<apollo::cyber::message::PyMessageWrap,
                                    apollo::cyber::message::PyMessageWrap>(
        service_name, f);
  }

  ~PyService() {}

  void register_func(int (*func)(const char *)) { func_ = func; }

  std::string read() {
    std::string msg("");
    if (!request_cache_.empty()) {
      msg = std::move(request_cache_.front());
      request_cache_.pop_front();
    }
    return msg;
  }

  int write(const std::string &data) {
    response_cache_.push_back(data);
    return SUCC;
  }

 private:
  void cb(const std::shared_ptr<const apollo::cyber::message::PyMessageWrap>
              &request,
          std::shared_ptr<apollo::cyber::message::PyMessageWrap>
              &response) {  // NOLINT
    std::lock_guard<std::mutex> lg(msg_lock_);

    request_cache_.push_back(request->data());

    if (func_) {
      func_(service_name_.c_str());
    }

    std::string msg("");
    if (!response_cache_.empty()) {
      msg = std::move(response_cache_.front());
      response_cache_.pop_front();
    }

    std::shared_ptr<apollo::cyber::message::PyMessageWrap> m;
    m.reset(new apollo::cyber::message::PyMessageWrap(msg, data_type_));
    response = m;
  }

  apollo::cyber::Node *node_;
  std::string service_name_;
  std::string data_type_;
  int (*func_)(const char *) = nullptr;
  std::shared_ptr<apollo::cyber::Service<apollo::cyber::message::PyMessageWrap,
                                         apollo::cyber::message::PyMessageWrap>>
      service_;
  std::mutex msg_lock_;
  std::deque<std::string> request_cache_;
  std::deque<std::string> response_cache_;
};

class PyClient {
 public:
  PyClient(const std::string &name, const std::string &data_type,
           apollo::cyber::Node *node)
      : node_(node), service_name_(name), data_type_(data_type) {
    client_ = node_->CreateClient<apollo::cyber::message::PyMessageWrap,
                                  apollo::cyber::message::PyMessageWrap>(name);
  }

  ~PyClient() {}

  std::string send_request(std::string request) {
    std::shared_ptr<apollo::cyber::message::PyMessageWrap> m;
    m.reset(new apollo::cyber::message::PyMessageWrap(request, data_type_));

    auto response = client_->SendRequest(m);
    if (response == nullptr) {
      AINFO << "SendRequest:response is null";
      return std::string("");
    }
    response->ParseFromString(response->data());

    return response->data();
  }

 private:
  apollo::cyber::Node *node_;
  std::string service_name_;
  std::string data_type_;
  std::shared_ptr<apollo::cyber::Client<apollo::cyber::message::PyMessageWrap,
                                        apollo::cyber::message::PyMessageWrap>>
      client_;
};

class PyNode {
 public:
  explicit PyNode(const std::string &node_name) : node_name_(node_name) {
    node_ = apollo::cyber::CreateNode(node_name);
  }
  ~PyNode() {}

  void shutdown() {
    node_.reset();
    AINFO << "PyNode " << node_name_ << " exit.";
  }

  PyWriter *create_writer(const std::string &channel, const std::string &type,
                          uint32_t qos_depth = 1) {
    if (node_) {
      return new PyWriter(channel, type, qos_depth, node_.get());
    }
    AINFO << "Py_Node: node_ is null, new PyWriter failed!";
    return nullptr;
  }

  void register_message(const std::string &desc) {
    apollo::cyber::message::ProtobufFactory::Instance()->RegisterPythonMessage(
        desc);
  }

  PyReader *create_reader(const std::string &channel, const std::string &type) {
    if (node_) {
      return new PyReader(channel, type, node_.get());
    }
    return nullptr;
  }

  PyService *create_service(const std::string &service,
                            const std::string &type) {
    if (node_) {
      return new PyService(service, type, node_.get());
    }
    return nullptr;
  }

  PyClient *create_client(const std::string &service, const std::string &type) {
    if (node_) {
      return new PyClient(service, type, node_.get());
    }
    return nullptr;
  }

 private:
  std::string node_name_;
  std::unique_ptr<apollo::cyber::Node> node_ = nullptr;
};

}  // namespace cyber
}  // namespace apollo

#endif  // PYTHON_WRAPPER_PY_NODE_H_
