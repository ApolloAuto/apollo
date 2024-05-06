/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/common/inference/model_util.h"

namespace apollo {
namespace perception {
namespace inference {

std::vector<std::string> GetBlobNames(
    const google::protobuf::RepeatedPtrField<common::ModelBlob>& model_blobs) {
  std::vector<std::string> blob_names;
  for (const auto& blob : model_blobs) {
    blob_names.push_back(blob.name());
  }
  return blob_names;
}

void AddShape(
    std::map<std::string, std::vector<int>>* shape_map,
    const google::protobuf::RepeatedPtrField<common::ModelBlob>& model_blobs) {
  for (const auto& blob : model_blobs) {
    std::vector<int> shape(blob.shape().begin(), blob.shape().end());
    shape_map->insert(std::make_pair(blob.name(), shape));
  }
}

}  // namespace inference
}  // namespace perception
}  // namespace apollo
