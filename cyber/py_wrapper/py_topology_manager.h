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

#include <iostream>
#include <string>
#include <memory>
#include <vector>
#include <algorithm>
#include "cyber/cyber.h"
#include "cyber/init.h"
#include "cyber/message/protobuf_factory.h"
#include "cyber/service_discovery/topology_manager.h"
#include "cyber/service_discovery/specific_manager/node_manager.h"

namespace apollo {
namespace cyber {

class PyNodeManager {
 public:
  PyNodeManager() {
     topology_ = apollo::cyber::service_discovery::TopologyManager::Instance();
     sleep(2);
     node_manager_ = topology_->node_manager();
     channel_manager_ = topology_->channel_manager();
  }

  int hasNode(const std::string& node_name) {
    return node_manager_->HasNode(node_name);
  }

  bool getNodes() {
    std::vector<RoleAttributes> nodes;
    topology_->node_manager()->GetNodes(&nodes);
    if (nodes.empty()) {
        AINFO << "no node found.";
        return 0;
    }
    std::sort(nodes.begin(), nodes.end(),
            [](const RoleAttributes& na, const RoleAttributes& nb) -> bool {
              if (na.node_name().compare(nb.node_name()) <= 0) {
                return true;
              } else {
                return false;
              }
            });
    std::cout << nodes.size() << std::endl;
    for (auto& node : nodes) {
        AINFO << "node list info get successful:" + node.node_name();
    }
    return true;
  }

  const char *connectChar(const char* temp1, const char* temp2) {
    const char* result = NULL;
    std::string temp = std::string(temp1) + std::string(temp2);
    result = temp.c_str();
    return result;
  }

  PyObject* showNodeInfo(const std::string& _node) {
      bool notFound = true;
      PyObject *pyobj_list = PyList_New(0);
      if (node_manager_->HasNode(_node)) {
        std::vector<RoleAttributes> nodes;
        node_manager_->GetNodes(&nodes);
        std::stringstream temps;
        for (auto& node : nodes) {
          if (node.node_name() == _node) {
            PyList_Append(pyobj_list, Py_BuildValue("s",
                connectChar("nodename:  ", node.node_name().c_str())));
            temps << node.process_id();
            PyList_Append(pyobj_list, Py_BuildValue("s",
                    connectChar("processid: ", temps.str().c_str())));
            PyList_Append(pyobj_list, Py_BuildValue("s",
                    connectChar("hostname:  ", node.host_name().c_str())));
            std::vector<RoleAttributes> readers;
            channel_manager_->GetReadersOfNode(_node, &readers);
            PyList_Append(pyobj_list, Py_BuildValue("s",
                    "[Reading Channels]:  "));
            for (auto& reader : readers) {
              if (reader.channel_name() == "param_event") {
                continue;
              }
              PyList_Append(pyobj_list, Py_BuildValue("s",
                    connectChar("    ", reader.channel_name().c_str())));
            }
            std::cout << std::endl;
            PyList_Append(pyobj_list, Py_BuildValue("s",
                    "[Writing Channels]:  "));
            std::vector<RoleAttributes> writers;
            channel_manager_->GetWritersOfNode(_node, &writers);
            for (auto& writer : writers) {
              if (writer.channel_name() == "param_event") {
                continue;
              }
              PyList_Append(pyobj_list, Py_BuildValue("s",
                    connectChar("    ", writer.channel_name().c_str())));
            }
            std::cout << std::endl;
            notFound = false;
          }
        }
      }
    if (notFound) {
        PyList_Append(pyobj_list, Py_BuildValue("s",
                    connectChar("Node cannot be found: ", _node.c_str())));
    }
    return pyobj_list;
  }

 private:
    apollo::cyber::service_discovery::TopologyManager* topology_ = nullptr;
    std::shared_ptr<apollo::cyber::service_discovery::NodeManager>
                                                            node_manager_;
    std::shared_ptr<apollo::cyber::service_discovery::ChannelManager>
                                                            channel_manager_;
};

}   // namespace cyber
}   // namespace apollo

#endif  // PYTHON_WRAPPER_PY_TOPOLOGY_MANAGER_H_
