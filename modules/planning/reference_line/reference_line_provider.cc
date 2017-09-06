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
 * @file reference_line_provider.cc
 *
 * @brief Implementation of the class ReferenceLineProvider.
 */

#include "modules/planning/reference_line/reference_line_provider.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

void ReferenceLineProvider::Init() {
  // TODO: implement this function.
  is_initialized_ = true;
}

bool ReferenceLineProvider::Start() {
  if (!is_initialized_) {
    AERROR << "LincolnController has NOT been initiated.";
    return false;
  }
  const auto &func = [this] { Generate(); };
  thread_.reset(new std::thread(func));
  return false;
}

void ReferenceLineProvider::Generate() {
  // TODO: implement this function.
}

std::vector<ReferenceLine> ReferenceLineProvider::GetReferenceLines() {
  // TODO: implement this function.
  return reference_line_groups_.back();
}

}  // namespace planning
}  // namespace apollo
