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
 */

#ifndef MODULES_PERCEPTION_PERCEPTION_H_
#define MODULES_PERCEPTION_PERCEPTION_H_

#include <string>

#include "modules/common/apollo_app.h"
#include "modules/perception/onboard/dag_streaming.h"

/**
 * @namespace apollo::perception
 * @brief apollo::perception
 */
namespace apollo {
namespace perception {

class Perception : public common::ApolloApp {
 public:
  std::string Name() const override;
  common::Status Init() override;
  common::Status Start() override;
  void Stop() override;

 private:
  DAGStreaming dag_streaming_;
  void RegistAllOnboardClass();
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_PERCEPTION_H_
