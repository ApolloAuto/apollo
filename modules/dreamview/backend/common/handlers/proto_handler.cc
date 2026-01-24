/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include <fstream>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/dreamview/backend/common/handlers/proto_handler.h"

namespace apollo {
namespace dreamview {

bool ProtoHandler::handleGet(CivetServer *server, struct mg_connection *conn) {
  const struct mg_request_info *req_info = mg_get_request_info(conn);

  // parse request rui
  std::string request_uri = req_info->local_uri;

  // replace /proto to actual file root path prefix,remove /proto
  // todo(@lijin):adapt to package version,change this to a variable
  std::string file_relative_path = request_uri.substr(6);
  std::string mime_type = mg_get_builtin_mime_type(request_uri.c_str());
  std::string content, response_header;

  {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    if (content_cache_.find(file_relative_path) != content_cache_.end()) {
      content = content_cache_[file_relative_path];
    }
  }

  if (content.empty()) {
    std::string file_abs_path;
    if (FindProtoPath(file_relative_path, &file_abs_path) &&
        apollo::cyber::common::GetContent(file_abs_path, &content)) {
      std::lock_guard<std::mutex> lock(cache_mutex_);
      content_cache_[file_relative_path] = content;
    } else {
      response_header = "HTTP/1.1 404 Not Found\r\nContent-Type: " + mime_type +
                        "\r\n\r\nFile not found";
      mg_printf(conn, response_header.c_str());
      return true;
    }
  }

  response_header = "HTTP/1.1 200 OK\r\nContent-Type: " + mime_type + "\r\n";
  mg_printf(conn, response_header.c_str());
  mg_printf(conn, "Cache-Control: max-age=86400\r\n\r\n");  // 缓存 24 小时
  // mg_printf(conn, "ETag: \"%s\"\r\n",
  //           GenerateETag(content).c_str());  // 生成并发送ETag
  mg_write(conn, content.data(), content.size());

  return true;
}

bool ProtoHandler::FindProtoPath(const std::string file_relative_path,
                                 std::string* file_abs_path) {
  std::string tmp_file_path;
  // source code
  tmp_file_path = "/apollo" + file_relative_path;
  if (apollo::cyber::common::PathExists(tmp_file_path)) {
    *file_abs_path = tmp_file_path;
    return true;
  }
  // package -source code
  tmp_file_path = "/apollo_workspace" + file_relative_path;
  if (apollo::cyber::common::PathExists(tmp_file_path)) {
    *file_abs_path = tmp_file_path;
    return true;
  }
  // package -source code
  tmp_file_path = "/opt/apollo/neo/src" + file_relative_path;
  if (apollo::cyber::common::PathExists(tmp_file_path)) {
    *file_abs_path = tmp_file_path;
    return true;
  }
  return false;
}

std::string ProtoHandler::GenerateETag(const std::string &content) {
  // 使用 std::hash 生成基于内容的哈希值
  std::hash<std::string> hasher;
  size_t hash = hasher(content);

  // 将哈希值转换为十六进制字符串
  std::stringstream ss;
  ss << std::hex << hash;

  return ss.str();
}

}  // namespace dreamview
}  // namespace apollo
