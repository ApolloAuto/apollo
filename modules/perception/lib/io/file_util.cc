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
#include "modules/perception/lib/io/file_util.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include <algorithm>
#include <cstring>
#include <fstream>

#include "cyber/common/log.h"
#include "modules/common/util/file.h"

namespace apollo {
namespace perception {
namespace lib {

using std::string;
using apollo::common::util::PathExists;
bool FileUtil::GetFileList(const std::string &path, const std::string &suffix,
                           std::vector<std::string> *files) {
  if (!PathExists(path)) {
    AINFO << path << " not exist.";
    return false;
  }

  boost::filesystem::recursive_directory_iterator itr(path);
  while (itr != boost::filesystem::recursive_directory_iterator()) {
    try {
      if (suffix.empty() ||
          boost::algorithm::ends_with(itr->path().string(), suffix)) {
        files->push_back(itr->path().string());
      }
      ++itr;
    } catch (const std::exception &ex) {
      AWARN << "Caught execption: " << ex.what();
      continue;
    }
  }
  return true;
}

string FileUtil::GetAbsolutePath(const string &prefix,
                                 const string &relative_path) {
  if (relative_path.empty()) {
    return prefix;
  }

  if (prefix.empty()) {
    return relative_path;
  }

  string result = prefix;

  if (relative_path[0] == '/') {
    return relative_path;
  }

  if (prefix[prefix.length() - 1] != '/') {
    result.append("/");
  }
  return result.append(relative_path);
}

}  // namespace lib
}  // namespace perception
}  // namespace apollo
