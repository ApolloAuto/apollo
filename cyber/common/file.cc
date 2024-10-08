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

#include "cyber/common/file.h"

#include <dirent.h>
#include <fcntl.h>
#include <glob.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cstddef>
#include <fstream>
#include <string>

#include "google/protobuf/util/json_util.h"
#include "nlohmann/json.hpp"

namespace apollo {
namespace cyber {
namespace common {

using std::istreambuf_iterator;
using std::string;
using std::vector;

bool SetProtoToASCIIFile(const google::protobuf::Message &message,
                         int file_descriptor) {
  using google::protobuf::TextFormat;
  using google::protobuf::io::FileOutputStream;
  using google::protobuf::io::ZeroCopyOutputStream;
  if (file_descriptor < 0) {
    AERROR << "Invalid file descriptor.";
    return false;
  }
  ZeroCopyOutputStream *output = new FileOutputStream(file_descriptor);
  bool success = TextFormat::Print(message, output);
  delete output;
  close(file_descriptor);
  return success;
}

bool SetProtoToASCIIFile(const google::protobuf::Message &message,
                         const std::string &file_name) {
  int fd = open(file_name.c_str(), O_WRONLY | O_CREAT | O_TRUNC, S_IRWXU);
  if (fd < 0) {
    AERROR << "Unable to open file " << file_name << " to write.";
    return false;
  }
  return SetProtoToASCIIFile(message, fd);
}

bool SetStringToASCIIFile(const std::string &content,
                          const std::string &file_name) {
  int fd = open(file_name.c_str(), O_WRONLY | O_CREAT | O_TRUNC, S_IRWXU);
  if (fd < 0) {
    AERROR << "Unable to open file " << file_name << " to write.";
    return false;
  }
  // Write the string data to the file
  ssize_t bytes_written = write(fd, content.c_str(), content.size());
  if (bytes_written < 0) {
    AERROR << "Failed to write to file.";
    close(fd);  // Ensure the file descriptor is closed even on error
    return false;
  }

  close(fd);  // Close the file descriptor

  return true;
}

bool GetProtoFromASCIIFile(const std::string &file_name,
                           google::protobuf::Message *message) {
  using google::protobuf::TextFormat;
  using google::protobuf::io::FileInputStream;
  using google::protobuf::io::ZeroCopyInputStream;
  int file_descriptor = open(file_name.c_str(), O_RDONLY);
  if (file_descriptor < 0) {
    AERROR << "Failed to open file " << file_name << " in text mode.";
    // Failed to open;
    return false;
  }

  ZeroCopyInputStream *input = new FileInputStream(file_descriptor);
  bool success = TextFormat::Parse(input, message);
  if (!success) {
    AERROR << "Failed to parse file " << file_name << " as text proto.";
  }
  delete input;
  close(file_descriptor);
  return success;
}

bool SetProtoToBinaryFile(const google::protobuf::Message &message,
                          const std::string &file_name) {
  std::fstream output(file_name,
                      std::ios::out | std::ios::trunc | std::ios::binary);
  return message.SerializeToOstream(&output);
}

bool GetProtoFromBinaryFile(const std::string &file_name,
                            google::protobuf::Message *message) {
  std::fstream input(file_name, std::ios::in | std::ios::binary);
  if (!input.good()) {
    AERROR << "Failed to open file " << file_name << " in binary mode.";
    return false;
  }
  if (!message->ParseFromIstream(&input)) {
    AERROR << "Failed to parse file " << file_name << " as binary proto.";
    return false;
  }
  return true;
}

bool GetProtoFromFile(const std::string &file_name,
                      google::protobuf::Message *message) {
  if (!PathExists(file_name)) {
    AERROR << "File [" << file_name << "] does not exist! ";
    return false;
  }
  // Try the binary parser first if it's much likely a binary proto.
  static const std::string kBinExt = ".bin";
  if (std::equal(kBinExt.rbegin(), kBinExt.rend(), file_name.rbegin())) {
    return GetProtoFromBinaryFile(file_name, message) ||
           GetProtoFromASCIIFile(file_name, message);
  }

  return GetProtoFromASCIIFile(file_name, message) ||
         GetProtoFromBinaryFile(file_name, message);
}

bool GetProtoFromJsonFile(const std::string &file_name,
                          google::protobuf::Message *message) {
  using google::protobuf::util::JsonParseOptions;
  using google::protobuf::util::JsonStringToMessage;
  std::ifstream ifs(file_name);
  if (!ifs.is_open()) {
    AERROR << "Failed to open file " << file_name;
    return false;
  }
  nlohmann::json Json;
  ifs >> Json;
  ifs.close();
  JsonParseOptions options;
  options.ignore_unknown_fields = true;
  google::protobuf::util::Status dump_status;
  return (JsonStringToMessage(Json.dump(), message, options).ok());
}

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

std::string GetAbsolutePath(const std::string &prefix,
                            const std::string &relative_path) {
  if (relative_path.empty()) {
    return prefix;
  }
  // If prefix is empty or relative_path is already absolute.
  if (prefix.empty() || relative_path.front() == '/') {
    return relative_path;
  }

  if (prefix.back() == '/') {
    return prefix + relative_path;
  }
  return prefix + "/" + relative_path;
}

bool PathExists(const std::string &path) {
  struct stat info;
  return stat(path.c_str(), &info) == 0;
}

bool PathIsAbsolute(const std::string &path) {
  if (path.empty()) {
    return false;
  }
  return path.front() == '/';
}

bool DirectoryExists(const std::string &directory_path) {
  struct stat info;
  return stat(directory_path.c_str(), &info) == 0 && (info.st_mode & S_IFDIR);
}

std::vector<std::string> Glob(const std::string &pattern) {
  glob_t globs = {};
  std::vector<std::string> results;
  if (glob(pattern.c_str(), GLOB_TILDE, nullptr, &globs) == 0) {
    for (size_t i = 0; i < globs.gl_pathc; ++i) {
      results.emplace_back(globs.gl_pathv[i]);
    }
  }
  globfree(&globs);
  return results;
}

bool CopyFile(const std::string &from, const std::string &to) {
  std::ifstream src(from, std::ios::binary);
  if (!src) {
    AWARN << "Source path could not be normally opened: " << from;
    std::string command = "cp -r " + from + " " + to;
    ADEBUG << command;
    const int ret = std::system(command.c_str());
    if (ret == 0) {
      ADEBUG << "Copy success, command returns " << ret;
      return true;
    } else {
      ADEBUG << "Copy error, command returns " << ret;
      return false;
    }
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
      const std::string sub_path_from = from + "/" + entry->d_name;
      const std::string sub_path_to = to + "/" + entry->d_name;
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

std::vector<std::string> ListSubPaths(const std::string &directory_path,
                                      const unsigned char d_type) {
  std::vector<std::string> result;
  DIR *directory = opendir(directory_path.c_str());
  if (directory == nullptr) {
    AERROR << "Cannot open directory " << directory_path;
    return result;
  }

  struct dirent *entry;
  while ((entry = readdir(directory)) != nullptr) {
    // Skip "." and "..".
    if (entry->d_type == d_type && strcmp(entry->d_name, ".") != 0 &&
        strcmp(entry->d_name, "..") != 0) {
      result.emplace_back(entry->d_name);
    }
  }
  closedir(directory);
  return result;
}

size_t FindPathByPattern(const std::string &base_path, const std::string &patt,
                         const unsigned char d_type, const bool recursive,
                         std::vector<std::string> *result_list) {
  DIR *directory = opendir(base_path.c_str());
  size_t result_cnt = 0;
  if (directory == nullptr) {
    AWARN << "cannot open directory " << base_path;
    return result_cnt;
  }
  struct dirent *entry;
  for (entry = readdir(directory); entry != nullptr;
       entry = readdir(directory)) {
    std::string entry_path = base_path + "/" + std::string(entry->d_name);
    // skip `.` and `..`
    if (strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0) {
      // TODO(liangjinping): support regex or glob or other pattern mode
      if ((patt == "" || strcmp(entry->d_name, patt.c_str()) == 0) &&
          entry->d_type == d_type) {
        // found
        result_list->emplace_back(entry_path);
        ++result_cnt;
      }
      if (recursive && (entry->d_type == DT_DIR)) {
        result_cnt +=
            FindPathByPattern(entry_path, patt, d_type, recursive, result_list);
      }
    }
  }
  closedir(directory);
  return result_cnt;
}

std::string GetDirName(const std::string &path) {
  std::string::size_type end = path.rfind('/');
  if (end == std::string::npos) {
    // not found, return current dir
    return ".";
  }
  return path.substr(0, end);
}

std::string GetFileName(const std::string &path, const bool remove_extension) {
  std::string::size_type start = path.rfind('/');
  if (start == std::string::npos) {
    start = 0;
  } else {
    // Move to the next char after '/'.
    ++start;
  }

  std::string::size_type end = std::string::npos;
  if (remove_extension) {
    end = path.rfind('.');
    // The last '.' is found before last '/', ignore.
    if (end != std::string::npos && end < start) {
      end = std::string::npos;
    }
  }
  const auto len = (end != std::string::npos) ? end - start : end;
  return path.substr(start, len);
}

bool GetFilePathWithEnv(const std::string &path, const std::string &env_var,
                        std::string *file_path) {
  if (path.empty()) {
    return false;
  }
  if (PathIsAbsolute(path)) {
    // an absolute path
    *file_path = path;
    return PathExists(path);
  }

  bool relative_path_exists = false;
  if (PathExists(path)) {
    // relative path exists
    *file_path = path;
    relative_path_exists = true;
  }
  if (path.front() == '.') {
    // relative path but not exist.
    return relative_path_exists;
  }

  const char *var = std::getenv(env_var.c_str());
  if (var == nullptr) {
    AWARN << "GetFilePathWithEnv: env " << env_var << " not found.";
    return relative_path_exists;
  }
  std::string env_path = std::string(var);

  // search by environment variable
  size_t begin = 0;
  size_t index;
  do {
    index = env_path.find(':', begin);
    auto p = env_path.substr(begin, index - begin);
    if (p.empty()) {
      continue;
    }
    if (p.back() != '/') {
      p += '/' + path;
    } else {
      p += path;
    }
    if (PathExists(p)) {
      *file_path = p;
      return true;
    }
    begin = index + 1;
  } while (index != std::string::npos);
  return relative_path_exists;
}

std::string GetCurrentPath() {
  char tmp[PATH_MAX];
  return getcwd(tmp, sizeof(tmp)) ? std::string(tmp) : std::string("");
}

bool GetType(const string &filename, FileType *type) {
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

bool DeleteFile(const string &filename) {
  if (!PathExists(filename)) {
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
  DIR *dir = opendir(filename.c_str());
  if (dir == nullptr) {
    AWARN << "failed to opendir: " << filename;
    return false;
  }
  dirent *dir_info = nullptr;
  while ((dir_info = readdir(dir)) != nullptr) {
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

bool CreateDir(const string &dir) {
  int ret = mkdir(dir.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);
  if (ret != 0) {
    AWARN << "failed to create dir. [dir: " << dir
          << "] [err: " << strerror(errno) << "]";
    return false;
  }
  return true;
}

}  // namespace common
}  // namespace cyber
}  // namespace apollo
