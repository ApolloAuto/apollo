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
 * @file reference_line_provider.h
 *
 * @brief Declaration of the class ReferenceLineProvider.
 */
#ifndef MODULES_PLANNING_REFERENCE_LINE_REFERENCE_LINE_PROVIDER_H_
#define MODULES_PLANNING_REFERENCE_LINE_REFERENCE_LINE_PROVIDER_H_

#include <memory>
#include <thread>
#include <vector>

#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/reference_line/reference_line_smoother.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class ReferenceLineProvider
 * @brief The class of ReferenceLineProvider.
 *        It provides smoothed reference line to planning.
 */
class ReferenceLineProvider {
 public:
  /**
   * @brief Default destructor.
   */
  virtual ~ReferenceLineProvider() = default;

  void Init();

  bool Start();

  std::vector<ReferenceLine> GetReferenceLines();

 private:
  DECLARE_SINGLETON(ReferenceLineProvider);

  void Generate();

  bool is_initialized_ = false;
  std::unique_ptr<std::thread> thread_;
  std::vector<std::vector<ReferenceLine>> reference_line_groups_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_REFERENCE_LINE_REFERENCE_LINE_PROVIDER_H_
