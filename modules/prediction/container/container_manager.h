/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

/**
 * @file
 * @brief Use container manager to manage all containers
 */

#ifndef MODULES_PREDICTION_CONTAINER_CONTAINER_MANAGER_H_
#define MODULES_PREDICTION_CONTAINER_CONTAINER_MANAGER_H_

#include <map>
#include <memory>
#include <string>

#include "modules/common/adapters/proto/adapter_config.pb.h"

#include "modules/common/macro.h"
#include "modules/prediction/container/container.h"

/**
 * @namespace apollo::prediction
 * @brief apollo::prediction
 */
namespace apollo {
namespace prediction {

class ContainerManager {
 public:
  /**
   * @brief Container manager initialization
   * @param Adapter config
   */
  void Init(const common::adapter::AdapterManagerConfig &config);

  /**
   * @brief Get mutable container
   * @param Type of the container
   * @return Pointer to the container given the name
   */
  Container *GetContainer(
      const common::adapter::AdapterConfig::MessageType &type);

 private:
  /**
   * @breif Register a container
   * @param Container type
   */
  void RegisterContainer(
      const common::adapter::AdapterConfig::MessageType &type);

  /**
   * @brief Create a container
   * @param Container type
   * @return Container pointer
   */
  std::unique_ptr<Container> CreateContainer(
      const common::adapter::AdapterConfig::MessageType &type);

  /**
   * @brief Register all containers
   */
  void RegisterContainers();

 private:
  std::map<common::adapter::AdapterConfig::MessageType,
           std::unique_ptr<Container>>
      containers_;

  common::adapter::AdapterManagerConfig config_;

  DECLARE_SINGLETON(ContainerManager)
};

}  // namespace prediction
}  // namespace apollo

#endif  // MODULES_PREDICTION_CONTAINER_CONTAINER_MANAGER_H_
