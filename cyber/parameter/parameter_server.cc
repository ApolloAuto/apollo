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
#include "cyber/parameter/parameter_server.h"

#include "cyber/common/log.h"
#include "cyber/node/node.h"
#include "cyber/parameter/parameter_service_names.h"

namespace apollo {
namespace cyber {

ParameterServer::ParameterServer(const std::shared_ptr<Node>& node)
    : node_(node) {
  auto name = node_->Name();
  get_parameter_service_ = node_->CreateService<ParamName, Param>(
      FixParameterServiceName(name, GET_PARAMETER_SERVICE_NAME),
      [this](const std::shared_ptr<ParamName>& request,
             std::shared_ptr<Param>& response) {
        std::lock_guard<std::mutex> lock(param_map_mutex_);
        if (param_map_.find(request->value()) != param_map_.end()) {
          response->CopyFrom(param_map_[request->value()]);
        } else {
          AINFO << "GetParameterService: [" << request->value() << "] not set";
        }
      });

  set_parameter_service_ = node_->CreateService<Param, BoolResult>(
      FixParameterServiceName(name, SET_PARAMETER_SERVICE_NAME),
      [this](const std::shared_ptr<Param>& request,
             std::shared_ptr<BoolResult>& response) {
        std::lock_guard<std::mutex> lock(param_map_mutex_);
        param_map_[request->name()] = *request;
        response->set_value(true);
      });

  list_parameters_service_ = node_->CreateService<NodeName, Params>(
      FixParameterServiceName(name, LIST_PARAMETERS_SERVICE_NAME),
      [this](const std::shared_ptr<NodeName>& request,
             std::shared_ptr<Params>& response) {
        std::lock_guard<std::mutex> lock(param_map_mutex_);
        for (auto& item : param_map_) {
          auto param = response->add_param();
          param->CopyFrom(item.second);
        }
      });
}

void ParameterServer::SetParameter(const Parameter& parameter) {
  std::lock_guard<std::mutex> lock(param_map_mutex_);
  param_map_[parameter.Name()] = parameter.ToProtoParam();
}

bool ParameterServer::GetParameter(const std::string& parameter_name,
                                   Parameter* parameter) {
  std::lock_guard<std::mutex> lock(param_map_mutex_);
  auto ite = param_map_.find(parameter_name);
  if (ite == param_map_.end()) {
    return false;
  }
  parameter->FromProtoParam(ite->second);
  return true;
}

void ParameterServer::ListParameters(std::vector<Parameter>* parameters) {
  std::lock_guard<std::mutex> lock(param_map_mutex_);
  for (auto& item : param_map_) {
    Parameter parameter;
    parameter.FromProtoParam(item.second);
    parameters->emplace_back(parameter);
  }
}

}  // namespace cyber
}  // namespace apollo
