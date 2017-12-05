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

#include "modules/perception/lib/base/file_util.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <algorithm>
#include <cstring>
#include <fstream>
#include <vector>

#include "modules/common/log.h"

namespace apollo {
namespace perception {

using std::count;
using std::istreambuf_iterator;
using std::string;
using std::vector;

bool FileUtil::GetType(const string& filename, FileType* type) {
  struct stat stat_buf;
  if (lstat(filename.c_str(), &stat_buf) != 0) {
    return false;
  }
  if (S_ISDIR(stat_buf.st_mode) != 0) {
    *type = TYPE_DIR;
  } else if (S_ISREG(stat_buf.st_mode) != 0) {
    *type = TYPE_FILE;
  } else {
    AWARN << "failed to get type: " << filename;
    return false;
  }
  return true;
}

bool FileUtil::DeleteFile(const string& filename) {
  if (!Exists(filename)) {
    return true;
  }
  FileType type;
  if (!GetType(filename, &type)) {
    return false;
  }
  if (type == TYPE_FILE) {
    if (remove(filename.c_str()) != 0) {
      AERROR << "failed to remove file: " << filename;
      return false;
    }
    return true;
  }

  DIR* dir = opendir(filename.c_str());
  if (dir == NULL) {
    AWARN << "failed to opendir: " << filename;
    return false;
  }
  dirent* dir_info = NULL;
  while ((dir_info = readdir(dir)) != NULL) {
    if (strcmp(dir_info->d_name, ".") == 0 ||
        strcmp(dir_info->d_name, "..") == 0) {
      continue;
    }
    string temp_file = filename + "/" + string(dir_info->d_name);
    FileType temp_type;
    if (!GetType(temp_file, &temp_type)) {
      AWARN << "failed to get file type: " << temp_file;
      closedir(dir);
      return false;
    }
    if (type == TYPE_DIR) {
      DeleteFile(temp_file);
    }
    remove(temp_file.c_str());
  }
  closedir(dir);
  remove(filename.c_str());

  return true;
}

bool FileUtil::Exists(const string& file) {
  int ret = access(file.c_str(), F_OK);
  if (ret != 0) {
    AINFO << "file not exist. file: " << file << " ret: " << strerror(errno);
    return false;
  }
  return true;
}

bool FileUtil::Exists(const string& path, const string& suffix) {
  boost::filesystem::recursive_directory_iterator itr(path);
  while (itr != boost::filesystem::recursive_directory_iterator()) {
    if (boost::algorithm::ends_with(itr->path().string(), suffix)) {
      return true;
    }
    ++itr;
  }
  return false;
}

bool FileUtil::CreateDir(const string& dir) {
  int ret = mkdir(dir.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);
  if (ret != 0) {
    AWARN << "failed to create dir. [dir: " << dir
          << "] [err: " << strerror(errno) << "]";
    return false;
  }
  return true;
}

bool FileUtil::GetFileContent(const string& path, string* content) {
  if (content == NULL) {
    return false;
  }

  int fd = ::open(path.c_str(), O_RDONLY);
  if (fd < 0) {
    AWARN << "failed to open file: " << path;
    return false;
  }
  struct stat buf;
  if (::fstat(fd, &buf) != 0) {
    AWARN << "failed to lstat file: " << path;
    ::close(fd);
    return false;
  }

  size_t fsize = buf.st_size;
  content->resize(fsize);
  char* data = const_cast<char*>(content->data());
  int size = 0;
  size_t has_read = 0;
  do {
    size = ::read(fd, data + has_read, fsize - has_read);
    if (size < 0) {
      AWARN << "failed to read file: " << path;
      ::close(fd);
      return false;
    }
    has_read += size;
  } while (size > 0);
  ::close(fd);
  return true;
}

bool FileUtil::ReadLines(const string& path, vector<string>* lines) {
  std::ifstream fin(path);
  if (!fin.good()) {
    LOG(ERROR) << "Failed to open path: " << path;
    return false;
  }
  if (lines == nullptr) {
    return false;
  }

  string line;
  while (std::getline(fin, line)) {
    lines->push_back(line);
  }
  return true;
}

std::string FileUtil::RemoveFileSuffix(std::string filename) {
  int first_index = filename.find_last_of("/");
  size_t last_index = filename.find_last_of(".");
  if (last_index == std::string::npos) {
    last_index = filename.length();
  }
  std::string raw_name =
      filename.substr(first_index + 1, last_index - first_index - 1);
  return raw_name;
}

void FileUtil::GetFileList(const std::string& path, const std::string& suffix,
                           std::vector<std::string>* files) {
  if (!Exists(path)) {
    AINFO << path << " not exist.";
    return;
  }

  boost::filesystem::recursive_directory_iterator itr(path);
  while (itr != boost::filesystem::recursive_directory_iterator()) {
    try {
      if (suffix.empty() ||
          boost::algorithm::ends_with(itr->path().string(), suffix)) {
        files->push_back(itr->path().string());
      }
      ++itr;
    } catch (const std::exception& ex) {
      AWARN << "Caught execption: " << ex.what();
      continue;
    }
  }
}

void FileUtil::GetFileList(const string& path, vector<string>* files) {
  GetFileList(path, "", files);
}

string FileUtil::GetAbsolutePath(const string& prefix,
                                 const string& relative_path) {
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

void FileUtil::GetFileName(const string& file, string* name) {
  size_t pos_left = file.find_last_of('/');
  size_t pos_right = file.find_last_of('.');
  if (pos_right == string::npos) {
    *name = file.substr(pos_left + 1);
  } else {
    *name = file.substr(pos_left + 1, pos_right - pos_left - 1);
  }
}

bool FileUtil::CompareFileByDigital(const string& file_left,
                                    const string& file_right) {
  return CompareFile(file_left, file_right, FCT_DIGITAL);
}

bool FileUtil::CompareFileByLexicographical(const string& file_left,
                                            const string& file_right) {
  return CompareFile(file_left, file_right, FCT_LEXICOGRAPHICAL);
}

// private functions

bool FileUtil::CompareFile(const string& file_left, const string& file_right,
                           FileCompareType type) {
  string name_left;
  GetFileName(file_left, &name_left);
  string name_right;
  GetFileName(file_right, &name_right);

  switch (type) {
    case FCT_DIGITAL:
      return atoll(name_left.c_str()) < atoll(name_right.c_str());
    case FCT_LEXICOGRAPHICAL:
      return std::lexicographical_compare(name_left.begin(), name_left.end(),
                                          name_right.begin(), name_right.end());
    default:
      AERROR << "Unknown compare type!";
  }

  return true;
}

int FileUtil::NumLines(const std::string& filename) {
  std::ifstream ifs(filename.c_str());
  return ifs.good()
             ? count(istreambuf_iterator<char>(ifs),
                     istreambuf_iterator<char>(), '\n') +
                   1
             : -1;
}

}  // namespace perception
}  // namespace apollo
