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

#include "cyber/parameter/parameter_client.h"
#include "cyber/node/node.h"
#include "cyber/parameter/parameter_service_names.h"

namespace apollo {
namespace cyber {

ParameterClient::ParameterClient(const std::shared_ptr<Node>& node,
                                 const std::string& service_node_name)
    : node_(node) {
  get_parameter_client_ = node_->CreateClient<ParamName, Param>(
      FixParameterServiceName(service_node_name, GET_PARAMETER_SERVICE_NAME));

  set_parameter_client_ = node_->CreateClient<Param, BoolResult>(
      FixParameterServiceName(service_node_name, SET_PARAMETER_SERVICE_NAME));

  list_parameters_client_ = node_->CreateClient<NodeName, Params>(
      FixParameterServiceName(service_node_name, LIST_PARAMETERS_SERVICE_NAME));
}

bool ParameterClient::GetParameter(const std::string& param_name,
                                   Parameter* parameter) {
  auto request = std::make_shared<ParamName>();
  request->set_value(param_name);
  auto response = get_parameter_client_->SendRequest(request);
  if (response == nullptr) {
    AERROR << "Call " << get_parameter_client_->ServiceName() << " failed";
    return false;
  }
  if (response->type() == ParamType::NOT_SET) {
    AWARN << "Parameter " << param_name << " not exists yet.";
    return false;
  }
  parameter->FromProtoParam(*response);
  return true;
}

bool ParameterClient::SetParameter(const Parameter& parameter) {
  auto request = std::make_shared<Param>(parameter.ToProtoParam());
  auto response = set_parameter_client_->SendRequest(request);
  if (response == nullptr) {
    AERROR << "Call " << set_parameter_client_->ServiceName() << " failed";
    return false;
  }
  return response->value();
}

bool ParameterClient::ListParameters(std::vector<Parameter>* parameters) {
  auto request = std::make_shared<NodeName>();
  request->set_value(node_->Name());
  auto response = list_parameters_client_->SendRequest(request);
  if (response == nullptr) {
    AERROR << "Call " << list_parameters_client_->ServiceName() << " failed";
    return false;
  }
  for (auto& param : response->param()) {
    Parameter parameter;
    parameter.FromProtoParam(param);
    parameters->emplace_back(parameter);
  }
  return true;
}

}  // namespace cyber
}  // namespace apollo
