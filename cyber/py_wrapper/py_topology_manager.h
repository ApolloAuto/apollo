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

#ifndef PYTHON_WRAPPER_PY_TOPOLOGY_MANAGER_H_
#define PYTHON_WRAPPER_PY_TOPOLOGY_MANAGER_H_

#include <algorithm>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include "cyber/cyber.h"
#include "cyber/init.h"
#include "cyber/message/protobuf_factory.h"
#include "cyber/service_discovery/specific_manager/node_manager.h"
#include "cyber/service_discovery/topology_manager.h"

namespace apollo {
namespace cyber {

class PyNodeManager {
 public:
  PyNodeManager() {
    topology_ = service_discovery::TopologyManager::Instance();
    sleep(2);
    node_manager_ = topology_->node_manager();
    channel_manager_ = topology_->channel_manager();
  }

  bool HasNode(const std::string& node_name) {
    return node_manager_->HasNode(node_name);
  }

  PyObject* GetNodeList() {
    std::vector<RoleAttributes> nodes;
    PyObject* pyobj_list = PyList_New(0);
    topology_->node_manager()->GetNodes(&nodes);
    if (nodes.empty()) {
      AERROR << "no node found.";
    } else {
      std::sort(nodes.begin(), nodes.end(),
                [](const RoleAttributes& na, const RoleAttributes& nb) -> bool {
                  if (na.node_name().compare(nb.node_name()) <= 0) {
                    return true;
                  } else {
                    return false;
                  }
                });
      AINFO << "get node list successful.";
      AINFO << "number of nodes:" << nodes.size();
      PyList_Append(pyobj_list, Py_BuildValue("s", "Node List: "));
      for (auto& node : nodes) {
        PyList_Append(
            pyobj_list,
            Py_BuildValue("s", ConnectChar("    ", node.node_name().c_str())));
      }
    }
    return pyobj_list;
  }

  const char* ConnectChar(const char* convert_temp1,
                          const char* convert_temp2) {
    const char* result = nullptr;
    std::string convert_temp =
        std::string(convert_temp1) + std::string(convert_temp2);
    result = convert_temp.c_str();
    return result;
  }

  PyObject* ShowNodeInfo(const std::string& node_name) {
    bool notFound = true;
    PyObject* pyobj_list = PyList_New(0);
    if (node_manager_->HasNode(node_name)) {
      std::vector<RoleAttributes> nodes;
      node_manager_->GetNodes(&nodes);
      std::stringstream convert_temp;
      for (auto& node : nodes) {
        if (node.node_name() == node_name) {
          PyList_Append(
              pyobj_list,
              Py_BuildValue(
                  "s", ConnectChar("nodename:  ", node.node_name().c_str())));
          convert_temp << node.process_id();
          PyList_Append(
              pyobj_list,
              Py_BuildValue(
                  "s", ConnectChar("processid: ", convert_temp.str().c_str())));
          convert_temp.clear();
          PyList_Append(
              pyobj_list,
              Py_BuildValue(
                  "s", ConnectChar("hostname:  ", node.host_name().c_str())));
          std::vector<RoleAttributes> readers;
          channel_manager_->GetReadersOfNode(node_name, &readers);
          PyList_Append(pyobj_list,
                        Py_BuildValue("s", "[Reading Channels]:  "));
          for (auto& reader : readers) {
            if (reader.channel_name() == "param_event") {
              continue;
            }
            PyList_Append(
                pyobj_list,
                Py_BuildValue(
                    "s", ConnectChar("    ", reader.channel_name().c_str())));
          }
          PyList_Append(pyobj_list,
                        Py_BuildValue("s", "[Writing Channels]:  "));
          std::vector<RoleAttributes> writers;
          channel_manager_->GetWritersOfNode(node_name, &writers);
          for (auto& writer : writers) {
            if (writer.channel_name() == "param_event") {
              continue;
            }
            PyList_Append(
                pyobj_list,
                Py_BuildValue(
                    "s", ConnectChar("    ", writer.channel_name().c_str())));
          }
          notFound = false;
        }
      }
    }
    if (notFound) {
      PyList_Append(pyobj_list,
                    Py_BuildValue("s", ConnectChar("Node cannot be found: ",
                                                   node_name.c_str())));
    }
    return pyobj_list;
  }

 private:
  service_discovery::TopologyManager* topology_ = nullptr;
  std::shared_ptr<service_discovery::NodeManager> node_manager_;
  std::shared_ptr<service_discovery::ChannelManager> channel_manager_;
};

class PyChannelManager {
 public:
  PyChannelManager() {
    topology_ = service_discovery::TopologyManager::Instance();
    sleep(2);
    node_manager_ = topology_->node_manager();
    channel_manager_ = topology_->channel_manager();
  }

  const char* ConnectChar(const char* convert_temp1,
                          const char* convert_temp2) {
    const char* result = nullptr;
    std::string convert_temp =
        std::string(convert_temp1) + std::string(convert_temp2);
    result = convert_temp.c_str();
    return result;
  }

  PyObject* GetChannelList() {
    PyObject* pyobj_list = PyList_New(0);
    std::vector<std::string> channels;
    channel_manager_->GetChannelNames(&channels);
    if (channels.empty()) {
      AERROR << "no channel found.";
    } else {
      AINFO << "get channel list successful.";
      AINFO << "number of channels:" << channels.size();
      std::sort(channels.begin(), channels.end());
      PyList_Append(pyobj_list, Py_BuildValue("s", "Channel List: "));
      for (auto& channel : channels) {
        PyList_Append(pyobj_list,
                      Py_BuildValue("s", ConnectChar("    ", channel.c_str())));
      }
    }
    return pyobj_list;
  }

  PyObject* GetReaderList() {
    PyObject* pyobj_list = PyList_New(0);
    std::vector<proto::RoleAttributes> readers;
    channel_manager_->GetReaders(&readers);
    if (readers.empty()) {
      AERROR << "no reader found.";
    } else {
      AINFO << "get reader list successful.";
      AINFO << "number of readers:" << readers.size();
      for (auto& reader : readers) {
        PyList_Append(pyobj_list,
                      Py_BuildValue("s", reader.channel_name().c_str()));
      }
    }
    return pyobj_list;
  }

  PyObject* GetWriterList() {
    PyObject* pyobj_list = PyList_New(0);
    std::vector<proto::RoleAttributes> writers;
    channel_manager_->GetWriters(&writers);
    if (writers.empty()) {
      AERROR << "no writer found.";
    } else {
      AINFO << "get writer list successful.";
      AINFO << "number of writers:" << writers.size();
      for (auto& writer : writers) {
        PyList_Append(pyobj_list,
                      Py_BuildValue("s", writer.channel_name().c_str()));
      }
    }
    return pyobj_list;
  }

  PyObject* GetWritersOfNode(const std::string& node_name) {
    PyObject* pyobj_list = PyList_New(0);
    std::vector<proto::RoleAttributes> writers;
    channel_manager_->GetWritersOfNode(node_name, &writers);
    if (writers.empty()) {
      AERROR << "no writer found for node:" << node_name;
    } else {
      AINFO << "get writers successful for nodes:" << node_name;
      AINFO << "number of writers:" << writers.size();
      for (auto& writer : writers) {
        PyList_Append(pyobj_list,
                      Py_BuildValue("s", writer.channel_name().c_str()));
      }
    }
    return pyobj_list;
  }

  PyObject* GetReadersOfNode(const std::string& node_name) {
    PyObject* pyobj_list = PyList_New(0);
    std::vector<proto::RoleAttributes> readers;
    channel_manager_->GetReadersOfNode(node_name, &readers);
    if (readers.empty()) {
      AERROR << "no reader found for node:" << node_name;
    } else {
      AINFO << "get readers successful for nodes:" << node_name;
      AINFO << "number of readers:" << readers.size();
      for (auto& reader : readers) {
        PyList_Append(pyobj_list,
                      Py_BuildValue("s", reader.channel_name().c_str()));
      }
    }
    return pyobj_list;
  }

  PyObject* ShowChannelInfo(const std::string& channel_name) {
    bool notFound = true;
    PyObject* pyobj_list = PyList_New(0);
    std::unordered_map<std::string, std::vector<proto::RoleAttributes>>
        roles_info;
    if (channel_manager_->HasWriter(channel_name) ||
        channel_manager_->HasReader(channel_name)) {
      std::vector<proto::RoleAttributes> tmpVec;
      channel_manager_->GetWriters(&tmpVec);
      std::stringstream convert_temp;
      for (auto& attr : tmpVec) {
        std::string channel_name = attr.channel_name();
        roles_info[channel_name].push_back(attr);
      }
      tmpVec.clear();
      channel_manager_->GetReaders(&tmpVec);
      for (auto& attr : tmpVec) {
        std::string channel_name = attr.channel_name();
        roles_info[channel_name].push_back(attr);
      }
      convert_temp.str("");
      convert_temp << roles_info.size();
      PyList_Append(
          pyobj_list,
          Py_BuildValue(
              "s", ConnectChar("channels size: ", convert_temp.str().c_str())));
      for (auto it = roles_info.begin(); it != roles_info.end(); ++it) {
        if (it->first != channel_name) {
          continue;
        }
        convert_temp.str("");
        convert_temp << it->first;
        PyList_Append(
            pyobj_list,
            Py_BuildValue("s", ConnectChar("channel name: ",
                                           convert_temp.str().c_str())));
        for (auto role_info : it->second) {
          convert_temp.str("");
          convert_temp << role_info.id();
          PyList_Append(
              pyobj_list,
              Py_BuildValue("s", ConnectChar("  roleid:      ",
                                             convert_temp.str().c_str())));
          PyList_Append(
              pyobj_list,
              Py_BuildValue("s", ConnectChar("    hostname:   ",
                                             role_info.host_name().c_str())));
          convert_temp.str("");
          convert_temp << role_info.process_id();
          PyList_Append(
              pyobj_list,
              Py_BuildValue("s", ConnectChar("    processid:  ",
                                             convert_temp.str().c_str())));
          PyList_Append(
              pyobj_list,
              Py_BuildValue("s", ConnectChar("    nodename:   ",
                                             role_info.node_name().c_str())));
          PyList_Append(
              pyobj_list,
              Py_BuildValue("s",
                            ConnectChar("    msgtype:    ",
                                        role_info.message_type().c_str())));
        }
      }
      notFound = false;
    }
    if (notFound) {
      PyList_Append(pyobj_list,
                    Py_BuildValue("s", ConnectChar("Channel cannot be found: ",
                                                   channel_name.c_str())));
    }
    return pyobj_list;
  }

 private:
  service_discovery::TopologyManager* topology_ = nullptr;
  std::shared_ptr<service_discovery::NodeManager> node_manager_;
  std::shared_ptr<service_discovery::ChannelManager> channel_manager_;
};

}  // namespace cyber
}  // namespace apollo

#endif  // PYTHON_WRAPPER_PY_TOPOLOGY_MANAGER_H_
