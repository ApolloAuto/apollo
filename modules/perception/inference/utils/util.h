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

#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "modules/perception/base/blob.h"
#include "modules/perception/base/image.h"

namespace apollo {
namespace perception {
namespace inference {

template <typename T>
void load_data(const std::string &filename, std::vector<T> *outputs) {
  std::ifstream ifs(filename, std::ifstream::in);

  if (ifs.good()) {
    outputs->clear();
    T output;
    while (ifs >> output) {
      outputs->push_back(output);
    }
    ifs.close();
  }
}

std::shared_ptr<float> load_binary_data(const std::string &filename);

bool write_result(const std::string &out_path,
                  const std::vector<float> &results);
bool write_result(const std::string &out_path,
                  const std::map<std::string, std::vector<float> > &results);
}  // namespace inference
}  // namespace perception
}  // namespace apollo
