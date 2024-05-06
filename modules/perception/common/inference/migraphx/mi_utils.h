/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
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

#include <string>

#include "google/protobuf/io/zero_copy_stream_impl.h"
#include "google/protobuf/text_format.h"

#include "modules/perception/common/proto/rt.pb.h"

namespace apollo {
namespace perception {
namespace inference {
bool ReadProtoFromTextFile(const std::string &filename,
                           google::protobuf::Message *proto);
bool ReadProtoFromBinaryFile(const std::string &filename,
                             google::protobuf::Message *proto);
bool loadNetParams(const std::string &param_file, NetParameter *param);
std::string locateFile(const std::string &path, const std::string &input);

}  // namespace inference
}  // namespace perception
}  // namespace apollo
