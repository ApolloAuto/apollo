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

#include "modules/localization/msf/common/util/file_utility.h"
#include <dirent.h>
#include <errno.h>
#include <limits.h>
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS
#include <algorithm>
#include <iostream>
#include "fastrtps/TopicDataType.h"

namespace apollo {
namespace localization {
namespace msf {
const size_t BUFFER_SIZE = 20480000;

void FileUtility::ComputeFileMd5(const std::string &file_path,
                                 unsigned char res[UCHAR_MD5LENTH]) {
  std::vector<unsigned char> buf(BUFFER_SIZE);
  unsigned char *buf_pt = &buf[0];

  FILE *file = fopen(file_path.c_str(), "rb");
  size_t total_size = 0;
  if (file) {
    int count = 1;
    while (!feof(file)) {
      if (count > 1) {
        buf.resize(BUFFER_SIZE * count);
        buf_pt = &buf[count - 1];
      }

      size_t size = fread(buf_pt, sizeof(unsigned char), BUFFER_SIZE, file);
      total_size += size;

      ++count;
    }
  } else {
    std::cerr << "Can't find the file: " << file_path << std::endl;
    return;
  }

  ComputeBinaryMd5(&buf[0], total_size, res);
  fclose(file);
}

void FileUtility::ComputeFileMd5(const std::string &file_path,
                                 char res[CHAR_MD5LENTH]) {
  std::vector<unsigned char> buf(BUFFER_SIZE);
  unsigned char *buf_pt = &buf[0];

  FILE *file = fopen(file_path.c_str(), "rb");
  size_t total_size = 0;
  if (file) {
    int count = 1;
    while (!feof(file)) {
      if (count > 1) {
        buf.resize(BUFFER_SIZE * count);
        buf_pt = &buf[count - 1];
      }

      size_t size = fread(buf_pt, sizeof(unsigned char), BUFFER_SIZE, file);
      total_size += size;

      ++count;
    }
  } else {
    std::cerr << "Can't find the file: " << file_path << std::endl;
    return;
  }

  ComputeBinaryMd5(&buf[0], total_size, res);
  fclose(file);
}

void FileUtility::ComputeBinaryMd5(const unsigned char *binary, size_t size,
                                   unsigned char res[UCHAR_MD5LENTH]) {
  MD5 md5;
  md5.init();
  md5.update(binary, static_cast<unsigned int>(size));
  md5.finalize();
  for (uint8_t i = 0; i < UCHAR_MD5LENTH; ++i) {
    res[i] = md5.digest[i];
  }
}

void FileUtility::ComputeBinaryMd5(const unsigned char *binary, size_t size,
                                   char res[CHAR_MD5LENTH]) {
  unsigned char md[UCHAR_MD5LENTH] = {"\0"};
  char buf[CHAR_MD5LENTH] = {'\0'};
  char tmp[3] = {'\0'};

  ComputeBinaryMd5(binary, size, md);

  for (unsigned int i = 0; i < UCHAR_MD5LENTH; i++) {
    snprintf(tmp, sizeof(tmp), "%02X", md[i]);
    strncat(buf, tmp, sizeof(tmp));
  }

  memcpy(res, buf, sizeof(buf));
}

bool FileUtility::GetContent(const std::string &file_name,
                             std::string *content) {
  std::ifstream fin(file_name);
  if (!fin) {
    return false;
  }

  std::stringstream str_stream;
  str_stream << fin.rdbuf();
  *content = str_stream.str();
  return true;
}

bool FileUtility::PathExists(const std::string &path) {
  struct stat info;
  return stat(path.c_str(), &info) == 0;
}

bool FileUtility::DirectoryExists(const std::string &directory_path) {
  struct stat info;
  if (stat(directory_path.c_str(), &info) != 0) {
    return false;
  }

  if (info.st_mode & S_IFDIR) {
    return true;
  }

  return false;
}

bool FileUtility::EnsureDirectory(const std::string &directory_path) {
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

bool FileUtility::RemoveAllFiles(const std::string &directory_path) {
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

std::vector<std::string> FileUtility::ListSubDirectories(
    const std::string &directory_path) {
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

bool FileUtility::IsExists(const std::string &path) {
  boost::filesystem::path p(path);
  return boost::filesystem::exists(p);
}
bool FileUtility::IsDirectory(const std::string &path) {
  boost::filesystem::path p(path);
  return boost::filesystem::is_directory(p);
}
bool FileUtility::CreateDirectory(const std::string &path) {
  boost::filesystem::path p(path);
  return boost::filesystem::create_directory(p);
}
bool FileUtility::GetFileSize(const std::string &path, size_t *size) {
  boost::filesystem::path p(path);
  if (boost::filesystem::exists(p)) {
    if (boost::filesystem::is_regular_file(p)) {
      *size = boost::filesystem::file_size(p);
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}

bool FileUtility::CopyFile(const std::string &src, const std::string &dst,
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
        path_src, path_dst,
        boost::filesystem::copy_option::overwrite_if_exists,
        error);
  }
  if (!error) {
    return true;
  } else {
    return false;
  }
}

void FileUtility::GetFilesInFolderRecursive(const std::string &folder,
                                            const std::string &ext,
                                            std::vector<std::string> *ret) {
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

void FileUtility::GetFilesInFolder(const std::string &folder,
                                   const std::string &ext,
                                   std::vector<std::string> *ret) {
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

void FileUtility::GetFoldersInFolder(const std::string &folder,
                                     std::vector<std::string> *ret) {
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
