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

#include "modules/common/util/file.h"

#include <dirent.h>
#include <errno.h>
#include <limits.h>
#include <algorithm>
#include <fstream>

#include "boost/filesystem.hpp"

#include "modules/common/util/string_util.h"

namespace apollo {
namespace common {
namespace util {
namespace {

std::string GetRosHome() {
  // Note that ROS_ROOT env points to <ROS_HOME>/share/ros.
  const std::string known_tail = "/share/ros";
  const std::string ros_root = CHECK_NOTNULL(std::getenv("ROS_ROOT"));
  CHECK(EndWith(ros_root, known_tail));
  return ros_root.substr(0, ros_root.length() - known_tail.length());
}

}  // namespace

bool GetContent(const std::string &file_name, std::string *content) {
  std::ifstream fin(file_name);
  if (!fin) {
    return false;
  }

  std::stringstream str_stream;
  str_stream << fin.rdbuf();
  *content = str_stream.str();
  return true;
}

std::string TranslatePath(const std::string &src_path) {
  static const std::string kRosHomePlaceHolder = "<ros>";
  static const std::string kRosHome = GetRosHome();

  std::string result(src_path);

  // Replace ROS home place holder.
  const auto pos = src_path.find(kRosHomePlaceHolder);
  if (pos != std::string::npos) {
    result.replace(pos, kRosHomePlaceHolder.length(), kRosHome);
  }

  return result;
}

std::string GetAbsolutePath(const std::string &prefix,
                            const std::string &relative_path) {
  if (relative_path.empty()) {
    return prefix;
  }
  // If prefix is empty or relative_path is already absolute.
  if (prefix.empty() || relative_path[0] == '/') {
    return relative_path;
  }

  if (prefix.back() == '/') {
    return StrCat(prefix, relative_path);
  }
  return StrCat(prefix, "/", relative_path);
}

bool PathExists(const std::string &path) {
  struct stat info;
  return stat(path.c_str(), &info) == 0;
}

bool DirectoryExists(const std::string &directory_path) {
  struct stat info;
  if (stat(directory_path.c_str(), &info) != 0) {
    return false;
  }

  if (info.st_mode & S_IFDIR) {
    return true;
  }

  return false;
}

bool CopyFile(const std::string &from, const std::string &to) {
  std::ifstream src(from, std::ios::binary);
  if (!src) {
    AERROR << "Source path doesn't exist: " << from;
    return false;
  }

  std::ofstream dst(to, std::ios::binary);
  if (!dst) {
    AERROR << "Target path is not writable: " << to;
    return false;
  }

  dst << src.rdbuf();
  return true;
}

bool CopyDir(const std::string &from, const std::string &to) {
  DIR *directory = opendir(from.c_str());
  if (directory == nullptr) {
    AERROR << "Cannot open directory " << from;
    return false;
  }

  bool ret = true;
  if (EnsureDirectory(to)) {
    struct dirent *entry;
    while ((entry = readdir(directory)) != nullptr) {
      // skip directory_path/. and directory_path/..
      if (!strcmp(entry->d_name, ".") || !strcmp(entry->d_name, "..")) {
        continue;
      }
      const std::string sub_path_from = StrCat(from, "/", entry->d_name);
      const std::string sub_path_to = StrCat(to, "/", entry->d_name);
      if (entry->d_type == DT_DIR) {
        ret &= CopyDir(sub_path_from, sub_path_to);
      } else {
        ret &= CopyFile(sub_path_from, sub_path_to);
      }
    }
  } else {
    AERROR << "Cannot create target directory " << to;
    ret = false;
  }
  closedir(directory);
  return ret;
}

bool Copy(const std::string &from, const std::string &to) {
  return DirectoryExists(from) ? CopyDir(from, to) : CopyFile(from, to);
}

bool EnsureDirectory(const std::string &directory_path) {
  std::string path = directory_path;
  for (size_t i = 1; i < directory_path.size(); ++i) {
    if (directory_path[i] == '/') {
      // Whenever a '/' is encountered, create a temporary view from
      // the start of the path to the character right before this.
      path[i] = 0;

      if (mkdir(path.c_str(), S_IRWXU) != 0) {
        if (errno != EEXIST) {
          return false;
        }
      }

      // Revert the temporary view back to the original.
      path[i] = '/';
    }
  }

  // Make the final (full) directory.
  if (mkdir(path.c_str(), S_IRWXU) != 0) {
    if (errno != EEXIST) {
      return false;
    }
  }

  return true;
}

bool RemoveAllFiles(const std::string &directory_path) {
  DIR *directory = opendir(directory_path.c_str());
  if (directory == nullptr) {
    AERROR << "Cannot open directory " << directory_path;
    return false;
  }

  struct dirent *file;
  while ((file = readdir(directory)) != nullptr) {
    // skip directory_path/. and directory_path/..
    if (!strcmp(file->d_name, ".") || !strcmp(file->d_name, "..")) {
      continue;
    }
    // build the path for each file in the folder
    std::string file_path = directory_path + "/" + file->d_name;
    if (unlink(file_path.c_str()) < 0) {
      AERROR << "Fail to remove file " << file_path << ": " << strerror(errno);
      closedir(directory);
      return false;
    }
  }
  closedir(directory);
  return true;
}

std::vector<std::string> ListSubDirectories(const std::string &directory_path) {
  std::vector<std::string> result;
  DIR *directory = opendir(directory_path.c_str());
  if (directory == nullptr) {
    AERROR << "Cannot open directory " << directory_path;
    return result;
  }

  struct dirent *entry;
  while ((entry = readdir(directory)) != nullptr) {
    // skip directory_path/. and directory_path/..
    if (!strcmp(entry->d_name, ".") || !strcmp(entry->d_name, "..")) {
      continue;
    }

    if (entry->d_type == DT_DIR) {
      result.emplace_back(entry->d_name);
    }
  }
  closedir(directory);
  return result;
}

std::string GetFileName(const std::string &path) {
  std::string filename;
  std::string::size_type loc = path.rfind('/');
  if (loc == std::string::npos) {
    filename = path;
  } else {
    filename = path.substr(loc + 1);
  }
  return filename;
}

void GetFileNamesInFolderById(const std::string &folder, const std::string &ext,
                              std::vector<std::string> *ret) {
  std::vector<double> ret_id;
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
      double temp_id = std::atof(temp_id_str.c_str());
      ret_id.push_back(temp_id);
    }
    ++it;
  }
  // sort
  int ret_size = ret->size();
  for (int i = 0; i < ret_size; ++i) {
    for (int j = i; j < ret_size; ++j) {
      if (ret_id[i] > ret_id[j]) {
        double temp_id = ret_id[i];
        ret_id[i] = ret_id[j];
        ret_id[j] = temp_id;
        std::string temp_path = (*ret)[i];
        (*ret)[i] = (*ret)[j];
        (*ret)[j] = temp_path;
      }
    }
  }
}

}  // namespace util
}  // namespace common
}  // namespace apollo
