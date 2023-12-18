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

#include <limits>
#include <map>
#include <memory>
#include <string>

#include "modules/perception/common/base/blob.h"

namespace apollo {
namespace perception {
namespace inference {

constexpr size_t kMaxStrLen = 64;
constexpr int kMinDim = 1;
constexpr int kMaxDim = std::numeric_limits<int>::max();

size_t BinaryReadString(FILE *fp, char *name);
size_t BinaryWriteString(FILE *fp, const std::string &str);

template <typename Dtype>
std::shared_ptr<base::Blob<Dtype>> BinaryReadBlob(FILE *fp);
template <typename Dtype>
void BinaryWriteBlob(FILE *fp, const base::Blob<Dtype> &blob);

template <typename Dtype>
std::map<std::string, std::shared_ptr<base::Blob<Dtype>>> BinaryReadFile(
    const char *file_path);
template <typename Btype>
bool BinaryWriteFile(const char *file_path,
                     const std::map<std::string, Btype> &data_dict);

}  // namespace inference
}  // namespace perception
}  // namespace apollo
