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
#include <vector>

#include "modules/common/util/file.h"
#include "modules/prediction/common/prediction_system_gflags.h"

namespace apollo {
namespace prediction {

Features FeatureOutput::features_;
ListDataForLearning FeatureOutput::list_data_for_learning_;
std::size_t FeatureOutput::idx_feature_ = 0;
std::size_t FeatureOutput::idx_learning_ = 0;

void FeatureOutput::Close() {
  ADEBUG << "Close feature output";
  if (FLAGS_prediction_offline_mode) {
    Write();
  }
  if (FLAGS_prediction_offline_dataforlearning) {
    WriteDataForLearning();
  }
  Clear();
}

void FeatureOutput::Clear() {
  idx_feature_ = 0;
  idx_learning_ = 0;
  features_.Clear();
  list_data_for_learning_.Clear();
}

bool FeatureOutput::Ready() {
  Clear();
  return true;
}

void FeatureOutput::Insert(const Feature& feature) {
  features_.add_feature()->CopyFrom(feature);
}

void FeatureOutput::InsertDataForLearning(
    const Feature& feature, const std::vector<double>& feature_values,
    const std::vector<double>& labels) {
  DataForLearning* data_for_learning =
      list_data_for_learning_.add_data_for_learning();
  data_for_learning->set_id(feature.id());
  data_for_learning->set_timestamp(feature.timestamp());
  for (size_t i = 0; i < feature_values.size(); ++i) {
    data_for_learning->add_features_for_learning(feature_values[i]);
  }
  for (size_t i = 0; i < labels.size(); ++i) {
    data_for_learning->add_labels(labels[i]);
  }
}

void FeatureOutput::Write() {
  if (features_.feature_size() <= 0) {
    ADEBUG << "Skip writing empty feature.";
  } else {
    const std::string file_name =
        FLAGS_prediction_data_dir + "/feature." +
        std::to_string(idx_feature_) + ".bin";
    common::util::SetProtoToBinaryFile(features_, file_name);
    features_.Clear();
    ++idx_feature_;
  }
}

void FeatureOutput::WriteDataForLearning() {
  if (list_data_for_learning_.data_for_learning_size() <= 0) {
    ADEBUG << "Skip writing empty data_for_learning.";
  } else {
    const std::string file_name =
        FLAGS_prediction_data_dir + "/datalearn." +
        std::to_string(idx_learning_) + ".bin";
    common::util::SetProtoToBinaryFile(list_data_for_learning_, file_name);
    list_data_for_learning_.Clear();
    ++idx_learning_;
  }
}

int FeatureOutput::Size() { return features_.feature_size(); }

int FeatureOutput::SizeOfDataForLearning() {
  return list_data_for_learning_.data_for_learning_size();
}

}  // namespace prediction
}  // namespace apollo
