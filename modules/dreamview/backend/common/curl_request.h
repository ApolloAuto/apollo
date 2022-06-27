/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include <future>
#include <map>
#include <string>

#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <curl/curl.h>

namespace apollo {
namespace dreamview {

class CurlRequest {
 public:
  CurlRequest();
  CurlRequest(const std::string url);
  CurlRequest(const std::string url,
              const std::map<std::string, std::string> queries);
  CurlRequest(const std::string url,
              const std::map<std::string, std::string> queries,
              const std::map<std::string, std::string> headers);
  ~CurlRequest();

  void AddParams(std::map<std::string, std::string> queries);
  void AddHeaders(std::map<std::string, std::string> headers);
  void AddFile(const std::string name, const std::string filepath);
  void AddFileFromString(const std::string name, const std::string filedata);
  bool SetEncodedCookie(const std::string key, const std::string val,
                        const std::string path, const std::string domain,
                        const int max_age);
  bool SetCookie(const std::string key, const std::string val,
                 const std::string path, const std::string domain,
                 const int max_age);
  void SetCookieFile(const std::string filepath);
  void SetCookieJar(const std::string filepath);
  void SetVerbose(long flag);
  void SetSslVerifyHost(long flag);
  void SetSslVerifyPeer(long flag);
  void SetUpload(long flag);
  void SetBody(const std::string body_);
  size_t WriteResponse(const char* ptr, size_t size, size_t nmemb);
  size_t WriteResponseHeader(const char* ptr, size_t size, size_t nitems);
  size_t ReadDataFromString(char* ptr, size_t size, size_t nmemb);
  int WriteProgress(double dltotal, double dlnow, double ultotal, double ulnow);
  void Abort();
  void SetCallback(std::function<void(int)> func);
  void Async();

  bool perform(std::function<void(int)> func);
  bool perform();
  const std::string Response();
  const std::map<std::string, std::string> ResponseHeaders();
  const std::string ResponseHeader(const std::string key);

 private:
  static void GlobalInit();
  static void GlobalClean();

  void Init();
  void AsyncPerform();
  void ProcessResponseHeaders();

  static int instance_count_;
  static boost::shared_mutex mutex_;
  std::string url_;
  CURL* inst_;
  CURLM* multi_handle_;
  curl_mime* form_;
  bool is_aborted_;
  bool is_async_;
  bool is_body_setted_;
  bool is_file_added_;
  int async_running_;
  std::future<void> async_polling_thread_;
  std::function<void(int)> callback_;
  double dltotal_;
  double dlnow_;
  double ultotal_;
  double ulnow_;
  struct curl_slist* headers_;
  std::string body_;
  size_t body_read_pos_;
  std::string response_;
  std::string response_headers_buf_;
  std::map<std::string, std::string> response_headers_;
};

}  // namespace dreamview
}  // namespace apollo
