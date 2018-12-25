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

#include "modules/prediction/common/feature_output.h"

#include <string>

#include "modules/common/util/file.h"
#include "modules/prediction/common/prediction_system_gflags.h"

namespace apollo {
namespace prediction {

Features FeatureOutput::features_;
DataForLearning FeatureOutput::data_for_learning_;
std::size_t FeatureOutput::index_ = 0;

void FeatureOutput::Close() {
  ADEBUG << "Close feature output";
  Write();
  Clear();
}

void FeatureOutput::Clear() {
  index_ = 0;
  features_.Clear();
  data_for_learning_.Clear();
}

bool FeatureOutput::Ready() {
  Clear();
  return true;
}

void FeatureOutput::Insert(const Feature& feature) {
  features_.add_feature()->CopyFrom(feature);
}

void FeatureOutput::InsertIntoLearningData(const Feature& feature) {
  data_for_learning_.set_id(feature.id());
  data_for_learning_.set_timestamp(feature.timestamp());

  for (int i = 0; i < feature.lane().lane_graph().lane_sequence_size();
       i ++) {
    DataForLearning::LaneSequenceData lane_sequence_data;
    auto curr_lane_sequence = feature.lane().lane_graph().lane_sequence(i);

    lane_sequence_data.set_lane_sequence_id(
        curr_lane_sequence.lane_sequence_id());
    for (int j = 0; j < curr_lane_sequence.features().mlp_features_size();
         j ++) {
      lane_sequence_data.add_features_lane_learning(
          curr_lane_sequence.features().mlp_features(j));
    }
    data_for_learning_.add_lane_sequence_data()->CopyFrom(lane_sequence_data);
  }
}

void FeatureOutput::Write() {
  if (features_.feature_size() <= 0) {
    ADEBUG << "Skip writing empty feature.";
  } else {
    const std::string file_name =
        FLAGS_prediction_data_dir + "/feature." +
        std::to_string(index_) + ".bin";
    common::util::SetProtoToBinaryFile(features_, file_name);
    features_.Clear();
    ++index_;
  }

  if (!data_for_learning_.has_id()) {
    ADEBUG << "Skip writing empty data_for_learning.";
  } else {
    const std::string file_name =
        FLAGS_prediction_data_dir + "/datalearn." +
        std::to_string(index_) + ".bin";
    common::util::SetProtoToBinaryFile(data_for_learning_, file_name);
    data_for_learning_.Clear();
  }
}

int FeatureOutput::Size() { return features_.feature_size(); }

}  // namespace prediction
}  // namespace apollo
