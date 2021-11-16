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

#ifndef CYBER_PARAMETER_PARAMETER_SERVER_H_
#define CYBER_PARAMETER_PARAMETER_SERVER_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "cyber/proto/parameter.pb.h"

#include "cyber/parameter/parameter.h"
#include "cyber/service/service.h"

namespace apollo {
namespace cyber {

class Node;

/**
 * @class ParameterServer
 * @brief Parameter Service is a very important function of auto-driving.
 * If you want to set a key-value, and hope other nodes to get the value,
 * Routing, sensor internal/external references are set by Parameter Service
 * ParameterServer can set a parameter, and then you can get/list
 * paramter(s) by start a ParameterClient to send responding request
 * @warning You should only have one ParameterServer works
 */
class ParameterServer {
 public:
  using Param = apollo::cyber::proto::Param;
  using NodeName = apollo::cyber::proto::NodeName;
  using ParamName = apollo::cyber::proto::ParamName;
  using BoolResult = apollo::cyber::proto::BoolResult;
  using Params = apollo::cyber::proto::Params;
  /**
   * @brief Construct a new ParameterServer object
   *
   * @param node shared_ptr of the node handler
   */
  explicit ParameterServer(const std::shared_ptr<Node>& node);

  /**
   * @brief Set the Parameter object
   *
   * @param parmeter parameter to be set
   */
  void SetParameter(const Parameter& parmeter);

  /**
   * @brief Get the Parameter object
   *
   * @param parameter_name name of the parameer want to get
   * @param parameter pointer to store parameter want to get
   * @return true get parameter success
   * @return false parameter not exists
   */
  bool GetParameter(const std::string& parameter_name, Parameter* parameter);

  /**
   * @brief get all the parameters
   *
   * @param parameters result Paramter vector
   */
  void ListParameters(std::vector<Parameter>* parameters);

 private:
  std::shared_ptr<Node> node_;
  std::shared_ptr<Service<ParamName, Param>> get_parameter_service_;
  std::shared_ptr<Service<Param, BoolResult>> set_parameter_service_;
  std::shared_ptr<Service<NodeName, Params>> list_parameters_service_;

  std::mutex param_map_mutex_;
  std::unordered_map<std::string, Param> param_map_;
};

}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_PARAMETER_PARAMETER_SERVER_H_
