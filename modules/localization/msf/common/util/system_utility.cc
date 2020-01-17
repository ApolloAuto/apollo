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

#include "modules/localization/msf/common/util/system_utility.h"

#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS

#include <algorithm>

namespace apollo {
namespace localization {
namespace msf {

bool system::IsExists(const std::string& path) {
  boost::filesystem::path p(path);
  return boost::filesystem::exists(p);
}

bool system::IsDirectory(const std::string& path) {
  boost::filesystem::path p(path);
  return boost::filesystem::is_directory(p);
}

bool system::CreateDirectory(const std::string& path) {
  boost::filesystem::path p(path);
  return boost::filesystem::create_directory(p);
}

bool system::GetFileSize(const std::string& path, unsigned int* size) {
  boost::filesystem::path p(path);
  if (boost::filesystem::exists(p) && boost::filesystem::is_regular_file(p)) {
    *size = static_cast<unsigned int>(boost::filesystem::file_size(p));
    return true;
  }

  return false;
}

bool system::CopyFile(const std::string& src, const std::string& dst,
                      bool is_overwrite) {
  boost::filesystem::path path_src(src);
  boost::filesystem::path path_dst(dst);
  boost::system::error_code error;
  if (is_overwrite) {
    boost::filesystem::copy_file(path_src, path_dst,
                                 boost::filesystem::copy_option::fail_if_exists,
                                 error);
  } else {
    boost::filesystem::copy_file(
        path_src, path_dst, boost::filesystem::copy_option::overwrite_if_exists,
        error);
  }

  if (error) {
    return false;
  }

  return true;
}

void system::GetFilesInFolderRecursive(const std::string& folder,
                                       const std::string& ext,
                                       std::vector<std::string>* ret) {
  ret->clear();
  namespace fs = boost::filesystem;
  if (!fs::exists(folder) || !fs::is_directory(folder)) {
    return;
  }

  fs::recursive_directory_iterator it(folder);
  fs::recursive_directory_iterator endit;

  while (it != endit) {
    if (fs::is_regular_file(*it) && it->path().extension() == ext) {
      ret->push_back(it->path().string());
    }
    ++it;
  }
  std::sort(ret->begin(), ret->end());
}

void system::GetFilesInFolder(const std::string& folder, const std::string& ext,
                              std::vector<std::string>* ret) {
  ret->clear();
  namespace fs = boost::filesystem;
  if (!fs::exists(folder) || !fs::is_directory(folder)) {
    return;
  }

  fs::directory_iterator it(folder);
  fs::directory_iterator endit;

  while (it != endit) {
    if (fs::is_regular_file(*it) && it->path().extension() == ext) {
      ret->push_back(it->path().string());
    }
    ++it;
  }
  std::sort(ret->begin(), ret->end());
}

void system::GetFoldersInFolder(const std::string& folder,
                                std::vector<std::string>* ret) {
  ret->clear();
  namespace fs = boost::filesystem;
  if (!fs::exists(folder) || !fs::is_directory(folder)) {
    return;
  }

  fs::directory_iterator it(folder);
  fs::directory_iterator endit;

  while (it != endit) {
    if (fs::is_directory(*it)) {
      ret->push_back(it->path().string());
    }
    ++it;
  }
  std::sort(ret->begin(), ret->end());
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
