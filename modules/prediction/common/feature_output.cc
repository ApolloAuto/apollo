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

#include "modules/common/log.h"
#include "modules/common/util/file.h"
#include "modules/common/util/string_util.h"
#include "modules/prediction/common/prediction_gflags.h"

namespace apollo {
namespace prediction {

Features FeatureOutput::features_;
size_t FeatureOutput::index_ = 0;

void FeatureOutput::Close() {
  ADEBUG << "Close feature output";
  Write();
  Clear();
}

void FeatureOutput::Clear() {
  index_ = 0;
  features_.Clear();
}

bool FeatureOutput::Ready() {
  Clear();
  return true;
}

void FeatureOutput::Insert(const Feature& feature) {
  features_.add_feature()->CopyFrom(feature);
}

void FeatureOutput::Write() {
  if (features_.feature_size() <= 0) {
    ADEBUG << "Skip writing empty feature.";
    return;
  }
  std::string file_name = ::apollo::common::util::StrCat(
      FLAGS_prediction_data_file_prefix, ".", std::to_string(index_), ".bin");
  ::apollo::common::util::SetProtoToBinaryFile(features_, file_name);
  features_.Clear();
  ++index_;
}

int FeatureOutput::Size() { return features_.feature_size(); }

}  // namespace prediction
}  // namespace apollo
