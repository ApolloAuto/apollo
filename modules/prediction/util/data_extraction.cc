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

#include "modules/prediction/util/data_extraction.h"

#include "cyber/common/log.h"

namespace apollo {
namespace prediction {

void GetRecordFileNames(const boost::filesystem::path& p,
                        std::vector<std::string>* record_files) {
  CHECK(record_files);
  if (!boost::filesystem::exists(p)) {
    return;
  }
  if (boost::filesystem::is_regular_file(p)) {
    AINFO << "Found record file: " << p.c_str();
    record_files->push_back(p.c_str());
    return;
  }
  if (boost::filesystem::is_directory(p)) {
    for (auto& entry : boost::make_iterator_range(
             boost::filesystem::directory_iterator(p), {})) {
      GetRecordFileNames(entry.path(), record_files);
    }
  }
}

}  // namespace prediction
}  // namespace apollo
