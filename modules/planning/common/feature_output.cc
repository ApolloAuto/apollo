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

#include "modules/planning/common/feature_output.h"

#include "absl/strings/str_cat.h"
#include "cyber/common/file.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

LearningData FeatureOutput::learning_data_;
int FeatureOutput::learning_data_file_index_ = 0;

void FeatureOutput::Close() { Clear(); }

void FeatureOutput::Clear() {
  learning_data_.Clear();
  learning_data_file_index_ = 0;
}

bool FeatureOutput::Ready() {
  Clear();
  return true;
}

void FeatureOutput::InsertLearningDataFrame(
    const std::string& record_file,
    const LearningDataFrame& learning_data_frame) {
  learning_data_.add_learning_data_frame()->CopyFrom(learning_data_frame);

  // write frames into a file
  if (learning_data_.learning_data_frame_size() >=
      FLAGS_learning_data_frame_num_per_file) {
    WriteLearningData(record_file);
  }
}

LearningDataFrame* FeatureOutput::GetLatestLearningDataFrame() {
  const int size = learning_data_.learning_data_frame_size();
  return size > 0 ? learning_data_.mutable_learning_data_frame(size - 1)
                  : nullptr;
}

void FeatureOutput::InsertPlanningResult() {}

void FeatureOutput::WriteLearningData(const std::string& record_file) {
  std::string src_file_name =
      record_file.substr(record_file.find_last_of("/") + 1);
  src_file_name = src_file_name.empty() ? "00000" : src_file_name;
  const std::string dest_file =
      absl::StrCat(FLAGS_planning_data_dir, "/", src_file_name, ".",
                   learning_data_file_index_, ".bin");
  cyber::common::SetProtoToBinaryFile(learning_data_, dest_file);
  // cyber::common::SetProtoToASCIIFile(learning_data_, dest_file + ".txt");
  learning_data_.Clear();
  learning_data_file_index_++;
}

void FeatureOutput::WriteRemainderiLearningData(
    const std::string& record_file) {
  if (learning_data_.learning_data_frame_size() > 0) {
    WriteLearningData(record_file);
  }
}

}  // namespace planning
}  // namespace apollo
