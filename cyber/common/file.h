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

/**
 * @file
 */

#ifndef CYBER_COMMON_FILE_H_
#define CYBER_COMMON_FILE_H_

#include <dirent.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "google/protobuf/io/zero_copy_stream_impl.h"
#include "google/protobuf/text_format.h"

#include "cyber/common/log.h"

/**
 * @namespace apollo::common::util
 * @brief apollo::common::util
 */
namespace apollo {
namespace cyber {
namespace common {

// file type: file or directory
enum FileType { TYPE_FILE, TYPE_DIR };

bool SetProtoToASCIIFile(const google::protobuf::Message &message,
                         int file_descriptor);
/**
 * @brief Sets the content of the file specified by the file_name to be the
 *        ascii representation of the input protobuf.
 * @param message The proto to output to the specified file.
 * @param file_name The name of the target file to set the content.
 * @return If the action is successful.
 */
bool SetProtoToASCIIFile(const google::protobuf::Message &message,
                         const std::string &file_name);

/**
 * @brief Parses the content of the file specified by the file_name as ascii
 *        representation of protobufs, and merges the parsed content to the
 *        proto.
 * @param file_name The name of the file to parse whose content.
 * @param message The proto to carry the parsed content in the specified file.
 * @return If the action is successful.
 */
bool GetProtoFromASCIIFile(const std::string &file_name,
                           google::protobuf::Message *message);

/**
 * @brief Sets the content of the file specified by the file_name to be the
 *        binary representation of the input protobuf.
 * @param message The proto to output to the specified file.
 * @param file_name The name of the target file to set the content.
 * @return If the action is successful.
 */
bool SetProtoToBinaryFile(const google::protobuf::Message &message,
                          const std::string &file_name);

/**
 * @brief Parses the content of the file specified by the file_name as binary
 *        representation of protobufs, and merges the parsed content to the
 *        proto.
 * @param file_name The name of the file to parse whose content.
 * @param message The proto to carry the parsed content in the specified file.
 * @return If the action is successful.
 */
bool GetProtoFromBinaryFile(const std::string &file_name,
                            google::protobuf::Message *message);

/**
 * @brief Parses the content of the file specified by the file_name as a
 *        representation of protobufs, and merges the parsed content to the
 *        proto.
 * @param file_name The name of the file to parse whose content.
 * @param message The proto to carry the parsed content in the specified file.
 * @return If the action is successful.
 */
bool GetProtoFromFile(const std::string &file_name,
                      google::protobuf::Message *message);

/**
 * @brief Get file content as string.
 * @param file_name The name of the file to read content.
 * @param content The file content.
 * @return If the action is successful.
 */
bool GetContent(const std::string &file_name, std::string *content);

/**
 * @brief Get absolute path by concatenating prefix and relative_path.
 * @return The absolute path.
 */
std::string GetAbsolutePath(const std::string &prefix,
                            const std::string &relative_path);

/**
 * @brief Check if the path exists.
 * @param path a file name, such as /a/b/c.txt
 * @return If the path exists.
 */
bool PathExists(const std::string &path);

/**
 * @brief Check if the directory specified by directory_path exists
 *        and is indeed a directory.
 * @param directory_path Directory path.
 * @return If the directory specified by directory_path exists
 *         and is indeed a directory.
 */
bool DirectoryExists(const std::string &directory_path);

/**
 * @brief Expand path pattern to matched paths.
 * @param pattern Path pattern, which may contain wildcards [?*].
 * @return Matched path list.
 */
std::vector<std::string> Glob(const std::string &pattern);

/**
 * @brief Copy a file.
 * @param from The file path to copy from.
 * @param to The file path to copy to.
 * @return If the action is successful.
 */
bool CopyFile(const std::string &from, const std::string &to);

/**
 * @brief Copy a directory.
 * @param from The path to copy from.
 * @param to The path to copy to.
 * @return If the action is successful.
 */
bool CopyDir(const std::string &from, const std::string &to);

/**
 * @brief Copy a file or directory.
 * @param from The path to copy from.
 * @param to The path to copy to.
 * @return If the action is successful.
 */
bool Copy(const std::string &from, const std::string &to);

/**
 * @brief Check if a specified directory specified by directory_path exists.
 *        If not, recursively create the directory (and its parents).
 * @param directory_path Directory path.
 * @return If the directory does exist or its creation is successful.
 */
bool EnsureDirectory(const std::string &directory_path);

/**
 * @brief Remove all the files under a specified directory. Note that
 *        sub-directories are NOT affected.
 * @param directory_path Directory path.
 * @return If the action is successful.
 */
bool RemoveAllFiles(const std::string &directory_path);

/**
 * @brief List sub-paths.
 * @param directory_path Directory path.
 * @param d_type Sub-path type, DT_DIR for directory, or DT_REG for file.
 * @return A vector of sub-paths, without the directory_path prefix.
 */
std::vector<std::string> ListSubPaths(const std::string &directory_path,
                                      const unsigned char d_type = DT_DIR);

std::string GetFileName(const std::string &path,
                        const bool remove_extension = false);

std::string GetCurrentPath();

// delete file including file or directory
bool DeleteFile(const std::string &filename);

bool GetType(const std::string &filename, FileType *type);

bool CreateDir(const std::string &dir);
}  // namespace common
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_COMMON_FILE_H_
