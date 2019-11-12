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

#ifndef CYBER_PY_WRAPPER_PY_CYBER_H_
#define CYBER_PY_WRAPPER_PY_CYBER_H_

#include <unistd.h>

#include <algorithm>
#include <deque>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "cyber/cyber.h"
#include "cyber/init.h"
#include "cyber/message/protobuf_factory.h"
#include "cyber/message/py_message.h"
#include "cyber/message/raw_message.h"
#include "cyber/node/node.h"
#include "cyber/node/reader.h"
#include "cyber/node/writer.h"
#include "cyber/service_discovery/specific_manager/node_manager.h"

namespace apollo {
namespace cyber {

inline bool py_init(const std::string& module_name) {
  static bool inited = false;
  if (inited) {
    AINFO << "cyber already inited.";
    return true;
  }

  if (!Init(module_name.c_str())) {
    AERROR << "cyber::Init failed:" << module_name;
    return false;
  }
  inited = true;
  AINFO << "cyber init succ.";
  return true;
}

inline bool py_ok() { return OK(); }

inline void py_shutdown() { return Clear(); }

inline bool py_is_shutdown() { return IsShutdown(); }

inline void py_waitforshutdown() { return WaitForShutdown(); }

class PyWriter {
 public:
  PyWriter(const std::string& channel, const std::string& type,
           const uint32_t qos_depth, Node* node)
      : channel_name_(channel),
        data_type_(type),
        qos_depth_(qos_depth),
        node_(node) {
    std::string proto_desc;
    message::ProtobufFactory::Instance()->GetDescriptorString(type,
                                                              &proto_desc);
    if (proto_desc.empty()) {
      AWARN << "cpp can't find proto_desc msgtype->" << data_type_;
      return;
    }
    proto::RoleAttributes role_attr;
    role_attr.set_channel_name(channel_name_);
    role_attr.set_message_type(data_type_);
    role_attr.set_proto_desc(proto_desc);
    auto qos_profile = role_attr.mutable_qos_profile();
    qos_profile->set_depth(qos_depth_);
    writer_ = node_->CreateWriter<message::PyMessageWrap>(role_attr);
  }

  int write(const std::string& data) {
    auto message = std::make_shared<message::PyMessageWrap>(data, data_type_);
    message->set_type_name(data_type_);
    return writer_->Write(message);
  }

 private:
  std::string channel_name_;
  std::string data_type_;
  uint32_t qos_depth_;
  Node* node_ = nullptr;
  std::shared_ptr<Writer<message::PyMessageWrap>> writer_;
};

const char RAWDATATYPE[] = "RawData";
class PyReader {
 public:
  PyReader(const std::string& channel, const std::string& type, Node* node)
      : channel_name_(channel), data_type_(type), node_(node), func_(nullptr) {
    if (data_type_.compare(RAWDATATYPE) == 0) {
      auto f =
          [this](const std::shared_ptr<const message::PyMessageWrap>& request) {
            this->cb(request);
          };
      reader_ = node_->CreateReader<message::PyMessageWrap>(channel, f);
    } else {
      auto f =
          [this](const std::shared_ptr<const message::RawMessage>& request) {
            this->cb_rawmsg(request);
          };
      reader_rawmsg_ = node_->CreateReader<message::RawMessage>(channel, f);
    }
  }

  void register_func(int (*func)(const char*)) { func_ = func; }

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
  void cb(const std::shared_ptr<const message::PyMessageWrap>& message) {
    {
      std::lock_guard<std::mutex> lg(msg_lock_);
      cache_.push_back(message->data());
    }
    if (func_) {
      func_(channel_name_.c_str());
    }
    msg_cond_.notify_one();
  }

  void cb_rawmsg(const std::shared_ptr<const message::RawMessage>& message) {
    {
      std::lock_guard<std::mutex> lg(msg_lock_);
      cache_.push_back(message->message);
    }
    if (func_) {
      func_(channel_name_.c_str());
    }
    msg_cond_.notify_one();
  }

  std::string channel_name_;
  std::string data_type_;
  Node* node_ = nullptr;
  int (*func_)(const char*) = nullptr;
  std::shared_ptr<Reader<message::PyMessageWrap>> reader_ = nullptr;
  std::deque<std::string> cache_;
  std::mutex msg_lock_;
  std::condition_variable msg_cond_;

  std::shared_ptr<Reader<message::RawMessage>> reader_rawmsg_ = nullptr;
};

using PyMsgWrapPtr = std::shared_ptr<message::PyMessageWrap>;
class PyService {
 public:
  PyService(const std::string& service_name, const std::string& data_type,
            Node* node)
      : node_(node),
        service_name_(service_name),
        data_type_(data_type),
        func_(nullptr) {
    auto f = [this](
                 const std::shared_ptr<const message::PyMessageWrap>& request,
                 std::shared_ptr<message::PyMessageWrap>& response) {
      response = this->cb(request);
    };
    service_ =
        node_->CreateService<message::PyMessageWrap, message::PyMessageWrap>(
            service_name, f);
  }

  void register_func(int (*func)(const char*)) { func_ = func; }

  std::string read() {
    std::string msg("");
    if (!request_cache_.empty()) {
      msg = std::move(request_cache_.front());
      request_cache_.pop_front();
    }
    return msg;
  }

  int write(const std::string& data) {
    response_cache_.push_back(data);
    return SUCC;
  }

 private:
  PyMsgWrapPtr cb(
      const std::shared_ptr<const message::PyMessageWrap>& request) {
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

    PyMsgWrapPtr response;
    response.reset(new message::PyMessageWrap(msg, data_type_));
    return response;
  }

  Node* node_;
  std::string service_name_;
  std::string data_type_;
  int (*func_)(const char*) = nullptr;
  std::shared_ptr<Service<message::PyMessageWrap, message::PyMessageWrap>>
      service_;
  std::mutex msg_lock_;
  std::deque<std::string> request_cache_;
  std::deque<std::string> response_cache_;
};

class PyClient {
 public:
  PyClient(const std::string& name, const std::string& data_type, Node* node)
      : node_(node), service_name_(name), data_type_(data_type) {
    client_ =
        node_->CreateClient<message::PyMessageWrap, message::PyMessageWrap>(
            name);
  }

  std::string send_request(std::string request) {
    std::shared_ptr<message::PyMessageWrap> m;
    m.reset(new message::PyMessageWrap(request, data_type_));

    auto response = client_->SendRequest(m);
    if (response == nullptr) {
      AINFO << "SendRequest:response is null";
      return std::string("");
    }
    response->ParseFromString(response->data());

    return response->data();
  }

 private:
  Node* node_;
  std::string service_name_;
  std::string data_type_;
  std::shared_ptr<Client<message::PyMessageWrap, message::PyMessageWrap>>
      client_;
};

class PyNode {
 public:
  explicit PyNode(const std::string& node_name) : node_name_(node_name) {
    node_ = CreateNode(node_name);
  }

  void shutdown() {
    node_.reset();
    AINFO << "PyNode " << node_name_ << " exit.";
  }

  PyWriter* create_writer(const std::string& channel, const std::string& type,
                          uint32_t qos_depth = 1) {
    if (node_) {
      return new PyWriter(channel, type, qos_depth, node_.get());
    }
    AINFO << "Py_Node: node_ is null, new PyWriter failed!";
    return nullptr;
  }

  void register_message(const std::string& desc) {
    message::ProtobufFactory::Instance()->RegisterPythonMessage(desc);
  }

  PyReader* create_reader(const std::string& channel, const std::string& type) {
    if (node_) {
      return new PyReader(channel, type, node_.get());
    }
    return nullptr;
  }

  PyService* create_service(const std::string& service,
                            const std::string& type) {
    if (node_) {
      return new PyService(service, type, node_.get());
    }
    return nullptr;
  }

  PyClient* create_client(const std::string& service, const std::string& type) {
    if (node_) {
      return new PyClient(service, type, node_.get());
    }
    return nullptr;
  }

  std::shared_ptr<Node> get_node() { return node_; }

 private:
  std::string node_name_;
  std::shared_ptr<Node> node_ = nullptr;
};

class PyChannelUtils {
 public:
  // Get debugstring of rawmsgdata
  // Pls make sure the msg_type of rawmsg is matching
  // Used in cyber_channel echo command
  static std::string get_debugstring_by_msgtype_rawmsgdata(
      const std::string& msg_type, const std::string& rawmsgdata) {
    if (msg_type.empty()) {
      AERROR << "parse rawmessage the msg_type is null";
      return "";
    }
    if (rawmsgdata.empty()) {
      AERROR << "parse rawmessage the rawmsgdata is null";
      return "";
    }

    if (raw_msg_class_ == nullptr) {
      auto rawFactory = message::ProtobufFactory::Instance();
      raw_msg_class_ = rawFactory->GenerateMessageByType(msg_type);
    }

    if (raw_msg_class_ == nullptr) {
      AERROR << "raw_msg_class_  is null";
      return "";
    }

    if (!raw_msg_class_->ParseFromString(rawmsgdata)) {
      AERROR << "Cannot parse the msg [ " << msg_type << " ]";
      return "";
    }

    return raw_msg_class_->DebugString();
  }

  static std::string get_msgtype_by_channelname(const std::string& channel_name,
                                                uint8_t sleep_s = 0) {
    if (channel_name.empty()) {
      AERROR << "channel_name is null";
      return "";
    }
    auto topology = service_discovery::TopologyManager::Instance();
    sleep(sleep_s);
    auto channel_manager = topology->channel_manager();
    std::string msg_type("");
    channel_manager->GetMsgType(channel_name, &msg_type);
    return msg_type;
  }

  static std::vector<std::string> get_active_channels(uint8_t sleep_s = 2) {
    auto topology = service_discovery::TopologyManager::Instance();
    sleep(sleep_s);
    auto channel_manager = topology->channel_manager();
    std::vector<std::string> channels;
    channel_manager->GetChannelNames(&channels);
    return channels;
  }

  static std::unordered_map<std::string, std::vector<std::string>>
  get_channels_info(uint8_t sleep_s = 2) {
    auto topology = service_discovery::TopologyManager::Instance();
    sleep(sleep_s);
    std::vector<proto::RoleAttributes> tmpVec;
    topology->channel_manager()->GetWriters(&tmpVec);
    std::unordered_map<std::string, std::vector<std::string>> roles_info;

    for (auto& attr : tmpVec) {
      std::string channel_name = attr.channel_name();
      std::string msgdata;
      attr.SerializeToString(&msgdata);
      roles_info[channel_name].emplace_back(msgdata);
    }

    tmpVec.clear();
    topology->channel_manager()->GetReaders(&tmpVec);
    for (auto& attr : tmpVec) {
      std::string channel_name = attr.channel_name();
      std::string msgdata;
      attr.SerializeToString(&msgdata);
      roles_info[channel_name].emplace_back(msgdata);
    }
    return roles_info;
  }

 private:
  static google::protobuf::Message* raw_msg_class_;
};

class PyNodeUtils {
 public:
  static std::vector<std::string> get_active_nodes(uint8_t sleep_s = 2) {
    auto topology = service_discovery::TopologyManager::Instance();
    sleep(sleep_s);
    std::vector<std::string> node_names;
    std::vector<RoleAttributes> nodes;
    topology->node_manager()->GetNodes(&nodes);
    if (nodes.empty()) {
      AERROR << "no node found.";
      return node_names;
    }

    std::sort(nodes.begin(), nodes.end(),
              [](const RoleAttributes& na, const RoleAttributes& nb) -> bool {
                return na.node_name().compare(nb.node_name()) <= 0;
              });
    for (auto& node : nodes) {
      node_names.emplace_back(node.node_name());
    }
    return node_names;
  }

  static std::string get_node_attr(const std::string& node_name,
                                   uint8_t sleep_s = 2) {
    auto topology = service_discovery::TopologyManager::Instance();
    sleep(sleep_s);

    if (!topology->node_manager()->HasNode(node_name)) {
      AERROR << "no node named: " << node_name;
      return "";
    }

    std::vector<RoleAttributes> nodes;
    topology->node_manager()->GetNodes(&nodes);
    std::string msgdata;
    for (auto& node_attr : nodes) {
      if (node_attr.node_name() == node_name) {
        node_attr.SerializeToString(&msgdata);
        return msgdata;
      }
    }
    return "";
  }

  static std::vector<std::string> get_readersofnode(
      const std::string& node_name, uint8_t sleep_s = 2) {
    std::vector<std::string> reader_channels;
    auto topology = service_discovery::TopologyManager::Instance();
    sleep(sleep_s);
    if (!topology->node_manager()->HasNode(node_name)) {
      AERROR << "no node named: " << node_name;
      return reader_channels;
    }

    std::vector<RoleAttributes> readers;
    auto channel_mgr = topology->channel_manager();
    channel_mgr->GetReadersOfNode(node_name, &readers);
    for (auto& reader : readers) {
      if (reader.channel_name() == "param_event") {
        continue;
      }
      reader_channels.emplace_back(reader.channel_name());
    }
    return reader_channels;
  }

  static std::vector<std::string> get_writersofnode(
      const std::string& node_name, uint8_t sleep_s = 2) {
    std::vector<std::string> writer_channels;
    auto topology = service_discovery::TopologyManager::Instance();
    sleep(sleep_s);
    if (!topology->node_manager()->HasNode(node_name)) {
      AERROR << "no node named: " << node_name;
      return writer_channels;
    }

    std::vector<RoleAttributes> writers;
    auto channel_mgr = topology->channel_manager();
    channel_mgr->GetWritersOfNode(node_name, &writers);
    for (auto& writer : writers) {
      if (writer.channel_name() == "param_event") {
        continue;
      }
      writer_channels.emplace_back(writer.channel_name());
    }
    return writer_channels;
  }
};

class PyServiceUtils {
 public:
  static std::vector<std::string> get_active_services(uint8_t sleep_s = 2) {
    auto topology = service_discovery::TopologyManager::Instance();
    sleep(sleep_s);
    std::vector<std::string> srv_names;
    std::vector<RoleAttributes> services;
    topology->service_manager()->GetServers(&services);
    if (services.empty()) {
      AERROR << "no service found.";
      return srv_names;
    }

    std::sort(services.begin(), services.end(),
              [](const RoleAttributes& sa, const RoleAttributes& sb) -> bool {
                return sa.service_name().compare(sb.service_name()) <= 0;
              });
    for (auto& service : services) {
      srv_names.emplace_back(service.service_name());
    }
    return srv_names;
  }

  static std::string get_service_attr(const std::string& service_name,
                                      uint8_t sleep_s = 2) {
    auto topology = service_discovery::TopologyManager::Instance();
    sleep(sleep_s);

    if (!topology->service_manager()->HasService(service_name)) {
      AERROR << "no service: " << service_name;
      return "";
    }

    std::vector<RoleAttributes> services;
    topology->service_manager()->GetServers(&services);
    std::string msgdata;
    for (auto& service_attr : services) {
      if (service_attr.service_name() == service_name) {
        service_attr.SerializeToString(&msgdata);
        return msgdata;
      }
    }
    return "";
  }
};

}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_PY_WRAPPER_PY_CYBER_H_
