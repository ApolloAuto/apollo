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

#include "modules/perception/obstacle/common/file_system_util.h"

#include <algorithm>

#include "boost/filesystem.hpp"

namespace apollo {
namespace perception {

std::string GetFileName(const std::string& path){
    std::string filename;
    std::string::size_type loc = path.rfind('/');
    if (loc == std::string::npos){
        filename = path;
    } else{
        filename = path.substr(loc+1);
    }
    return filename;
}

void GetFileNamesInFolderById(const std::string& folder, const std::string& ext,
                              std::vector<std::string>* ret) {
  std::vector<int> ret_id;
  ret->clear();
  namespace fs = boost::filesystem;
  if (!fs::exists(folder) || !fs::is_directory(folder)) {
    return;
  }

  fs::directory_iterator it(folder);
  fs::directory_iterator endit;

  while (it != endit) {
    if (fs::is_regular_file(*it) && it->path().extension() == ext) {
      std::string temp_path = it->path().filename().string();
      ret->push_back(temp_path);
      std::string temp_id_str =
          temp_path.substr(temp_path.rfind('_') + 1,
                           temp_path.rfind('.') - temp_path.rfind('_') - 1);
      int temp_id = std::atoi(temp_id_str.c_str());
      ret_id.push_back(temp_id);
    }
    ++it;
  }
  // sort
  int ret_size = ret->size();
  for (int i = 0; i < ret_size; ++i) {
    for (int j = i; j < ret_size; ++j) {
      if (ret_id[i] > ret_id[j]) {
        int temp_id = ret_id[i];
        ret_id[i] = ret_id[j];
        ret_id[j] = temp_id;
        std::string temp_path = (*ret)[i];
        (*ret)[i] = (*ret)[j];
        (*ret)[j] = temp_path;
      }
    }
  }
}

}  // namespace perception
}  // namespace apollo
