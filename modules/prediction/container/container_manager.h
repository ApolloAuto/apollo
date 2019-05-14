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

#pragma once

#include <memory>
#include <unordered_map>

#include "cyber/common/macros.h"
#include "gtest/gtest.h"

#include "modules/common/adapters/proto/adapter_config.pb.h"
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
  template <typename T>
  T *GetContainer(const common::adapter::AdapterConfig::MessageType &type) {
    auto key_type = static_cast<int>(type);
    if (containers_.find(key_type) != containers_.end()) {
      return static_cast<T *>(containers_[key_type].get());
    }
    return nullptr;
  }

  FRIEND_TEST(FeatureExtractorTest, junction);
  FRIEND_TEST(ScenarioManagerTest, run);

 private:
  /**
   * @brief Register a container
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
  std::unordered_map<int, std::unique_ptr<Container>> containers_;

  common::adapter::AdapterManagerConfig config_;

  DECLARE_SINGLETON(ContainerManager)
};

}  // namespace prediction
}  // namespace apollo
