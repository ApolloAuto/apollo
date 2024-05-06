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

#include "modules/planning/planning_base/common/learning_based_data.h"

#include "cyber/common/log.h"

namespace apollo {
namespace planning {

void LearningBasedData::Clear() { learning_data_.Clear(); }

void LearningBasedData::InsertLearningDataFrame(
    const LearningDataFrame& learning_data_frame) {
  learning_data_.add_learning_data_frame()->CopyFrom(learning_data_frame);

  while (learning_data_.learning_data_frame_size() > 10) {
    learning_data_.mutable_learning_data_frame()->DeleteSubrange(0, 1);
  }
}

LearningDataFrame* LearningBasedData::GetLatestLearningDataFrame() {
  const int size = learning_data_.learning_data_frame_size();
  return size > 0 ? learning_data_.mutable_learning_data_frame(size - 1)
                  : nullptr;
}

}  // namespace planning
}  // namespace apollo
