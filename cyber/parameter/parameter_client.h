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

#ifndef CYBER_PARAMETER_PARAMETER_CLIENT_H_
#define CYBER_PARAMETER_PARAMETER_CLIENT_H_

#include <memory>
#include <string>
#include <vector>

#include "cyber/proto/parameter.pb.h"

#include "cyber/parameter/parameter.h"
#include "cyber/service/client.h"

namespace apollo {
namespace cyber {

class Node;

/**
 * @class ParameterClient
 * @brief Parameter Client is used to set/get/list parameter(s)
 * by sending a request to ParameterServer
 */
class ParameterClient {
 public:
  using Param = apollo::cyber::proto::Param;
  using NodeName = apollo::cyber::proto::NodeName;
  using ParamName = apollo::cyber::proto::ParamName;
  using BoolResult = apollo::cyber::proto::BoolResult;
  using Params = apollo::cyber::proto::Params;
  using GetParameterClient = Client<ParamName, Param>;
  using SetParameterClient = Client<Param, BoolResult>;
  using ListParametersClient = Client<NodeName, Params>;
  /**
   * @brief Construct a new ParameterClient object
   *
   * @param node shared_ptr of the node handler
   * @param service_node_name node name which provide a param services
   */
  ParameterClient(const std::shared_ptr<Node>& node,
                  const std::string& service_node_name);

  /**
   * @brief Get the Parameter object
   *
   * @param param_name
   * @param parameter the pointer to store
   * @return true
   * @return false call service fail or timeout
   */
  bool GetParameter(const std::string& param_name, Parameter* parameter);

  /**
   * @brief Set the Parameter object
   *
   * @param parameter parameter to be set
   * @return true set parameter succues
   * @return false 1. call service timeout
   *               2. parameter not exists
   *               The corresponding log will be recorded at the same time
   */
  bool SetParameter(const Parameter& parameter);

  /**
   * @brief Get all the Parameter objects
   *
   * @param parameters pointer of vector to store all the parameters
   * @return true
   * @return false call service fail or timeout
   */
  bool ListParameters(std::vector<Parameter>* parameters);

 private:
  std::shared_ptr<Node> node_;
  std::shared_ptr<GetParameterClient> get_parameter_client_;
  std::shared_ptr<SetParameterClient> set_parameter_client_;
  std::shared_ptr<ListParametersClient> list_parameters_client_;
};

}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_PARAMETER_PARAMETER_CLIENT_H_
