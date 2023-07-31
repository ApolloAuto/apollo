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

#pragma once

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "google/protobuf/repeated_field.h"

#include "modules/perception/common/proto/model_info.pb.h"

namespace apollo {
namespace perception {
namespace inference {

std::vector<std::string> GetBlobNames(
    const google::protobuf::RepeatedPtrField<common::ModelBlob>& model_blobs);

void AddShape(
    std::map<std::string, std::vector<int>>* shape_map,
    const google::protobuf::RepeatedPtrField<common::ModelBlob>& model_blobs);

}  // namespace inference
}  // namespace perception
}  // namespace apollo
