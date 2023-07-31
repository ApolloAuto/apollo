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

#include "modules/perception/common/inference/utils/util.h"
#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace inference {

std::shared_ptr<float> load_binary_data(const std::string &filename) {
  std::ifstream ifs(filename, std::ifstream::binary);
  if (!ifs) {
    return nullptr;
  }

  ifs.seekg(0, ifs.end);
  int length = static_cast<int>(ifs.tellg() / sizeof(float));
  ifs.seekg(0, ifs.beg);
  std::shared_ptr<float> outputs;
  outputs.reset(new float[length]);
  ifs.read(reinterpret_cast<char *>(outputs.get()), sizeof(float) * length);
  ifs.close();
  return outputs;
}

bool write_result(const std::string &out_path,
                  const std::vector<float> &results) {
  std::ofstream outf(out_path, std::ios::binary | std::ios::out);
  if (!outf.is_open()) {
    AINFO << "Cannot open output file: " << out_path;
    return false;
  }
  outf.write(reinterpret_cast<const char *>(results.data()),
             sizeof(float) * results.size());
  outf.close();
  return true;
}

}  // namespace inference
}  // namespace perception
}  // namespace apollo
