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
 * @brief Output information of prediction evaluator submodule
 */

#pragma once

#include "modules/prediction/submodules/submodule_output.h"

namespace apollo {
namespace prediction {

class EvaluatorOutput {
 public:
  /**
   * @brief Constructor
   */
  EvaluatorOutput() = default;

  /**
   * @brief Constructor from SubmoduleOutput
   */
  explicit EvaluatorOutput(const SubmoduleOutput&& submodule_output);

  /**
   * @brief Destructor
   */
  virtual ~EvaluatorOutput() = default;

  /**
   * @brief Get submodule output
   * @return submodule output
   */
  const SubmoduleOutput& submodule_output() const;

 private:
  SubmoduleOutput submodule_output_;
};

}  // namespace prediction
}  // namespace apollo
