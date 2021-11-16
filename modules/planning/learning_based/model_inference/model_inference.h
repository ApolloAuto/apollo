/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
 * @brief Define the model inference base class
 */

#pragma once

#include <string>

#include "modules/planning/proto/learning_data.pb.h"
#include "modules/planning/proto/task_config.pb.h"

namespace apollo {
namespace planning {

class ModelInference {
 public:
  /**
   * @brief Constructor
   */
  explicit ModelInference(const LearningModelInferenceTaskConfig& config)
      : config_(config) {}

  /**
   * @brief Destructor
   */
  virtual ~ModelInference() = default;

  /**
   * @brief Get the name of model inference
   */
  virtual std::string GetName() = 0;

  /**
   * @brief load a learned model
   */
  virtual bool LoadModel() = 0;

  /**
   * @brief inference a learned model
   */
  virtual bool DoInference(LearningDataFrame* learning_data_frame) = 0;

 protected:
  LearningModelInferenceTaskConfig config_;
};

}  // namespace planning
}  // namespace apollo
