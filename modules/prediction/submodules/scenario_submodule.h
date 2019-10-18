/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
 * @brief Use scenario submodule to deal with scenario-related tasks
 */

#pragma once

#include <memory>
#include <string>

#include "cyber/component/component.h"

#include "modules/prediction/proto/submodule_messages.pb.h"

namespace apollo {
namespace prediction {

class ScenarioSubmodule : public cyber::Component<PredictionContainerMessage> {
 public:
  /**
   * @brief Destructor
   */
  ~ScenarioSubmodule();

  /**
   * @brief Get name of the node
   * @return Name of the node
   */
  std::string Name() const;

  /**
   * @brief Initialize the node
   * @return If initialized
   */
  bool Init() override;

  /**
   * @brief Data callback upon receiving a prediction container message.
   * @param Prediction container message.
   */
  bool Proc(const std::shared_ptr<PredictionContainerMessage>&) override;

 private:
  // TODO(kechxu) define storytelling reader
  // TODO(kechxu) define writer
};

CYBER_REGISTER_COMPONENT(ScenarioSubmodule)

}  // namespace prediction
}  // namespace apollo
