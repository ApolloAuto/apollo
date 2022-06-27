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

#include "modules/dreamview/backend/common/curl_request.h"

#include "cyber/common/log.h"
#include "cyber/task/task.h"

namespace apollo {
namespace dreamview {

namespace {

size_t CaptureCURLResult(void* ptr, size_t size, size_t nmemb, void* dist) {
  CurlRequest* ref = (CurlRequest*)dist;
  return ref->WriteResponse((char*)ptr, size, nmemb);
}

size_t CaptureCURLHeader(void* ptr, size_t size, size_t nitems, void* dist) {
  CurlRequest* ref = (CurlRequest*)dist;
  return ref->WriteResponseHeader((char*)ptr, size, nitems);
}

size_t SendBodyFromString(char* ptr, size_t size, size_t nmemb, void* dist) {
  CurlRequest* ref = (CurlRequest*)dist;
  return ref->ReadDataFromString(ptr, size, nmemb);
}

int CaptureCURLProgress(void* clientp, double dltotal, double dlnow,
                        double ultotal, double ulnow) {
  CurlRequest* ref = (CurlRequest*)clientp;
  return ref->WriteProgress(dltotal, dlnow, ultotal, ulnow);
}

}  // namespace

boost::shared_mutex CurlRequest::mutex_;
int CurlRequest::instance_count_ = 0;

void CurlRequest::GlobalInit() {
  boost::unique_lock<boost::shared_mutex> w_lock(mutex_);
  if (!instance_count_) {
    curl_global_init(CURL_GLOBAL_DEFAULT);
  }
  instance_count_++;
}

void CurlRequest::GlobalClean() {
  boost::unique_lock<boost::shared_mutex> w_lock(mutex_);
  instance_count_--;
  if (!instance_count_) {
    curl_global_cleanup();
  }
}

void CurlRequest::Init() {
  is_async_ = false;
  is_aborted_ = false;
  is_body_setted_ = false;
  is_file_added_ = false;
  headers_ = NULL;
  inst_ = curl_easy_init();
  if (inst_) {
    form_ = curl_mime_init(inst_);
  }
  // multi_handle_ = curl_multi_init();
}

CurlRequest::CurlRequest() {
  GlobalInit();
  Init();
}

CurlRequest::CurlRequest(const std::string url) {
  GlobalInit();
  Init();
  url_ = url;
  if (inst_) {
    curl_easy_setopt(inst_, CURLOPT_URL, url_.c_str());
  }
}

CurlRequest::CurlRequest(const std::string url,
                         const std::map<std::string, std::string> queries) {
  GlobalInit();
  Init();
  url_ = url;
  AddParams(queries);
}

CurlRequest::CurlRequest(const std::string url,
                         const std::map<std::string, std::string> queries,
                         const std::map<std::string, std::string> headers) {
  GlobalInit();
  Init();
  url_ = url;
  AddParams(queries);
  AddHeaders(headers);
}

CurlRequest::~CurlRequest() {
  if (is_async_) {
    async_running_ = 0;
    if (async_polling_thread_.valid()) {
      async_polling_thread_.get();
    }
    // curl_multi_remove_handle(multi_handle_, inst_);
  }
  if (inst_) {
    curl_easy_cleanup(inst_);
    curl_mime_free(form_);
    curl_slist_free_all(headers_);
  }
  // curl_multi_cleanup(multi_handle_);
  GlobalClean();
}

void CurlRequest::AddParams(std::map<std::string, std::string> queries) {
  bool query_pos_found = url_.find("?", 0) != std::string::npos;
  for (auto const& x : queries) {
    if (!query_pos_found) {
      url_.append("?").append(x.first).append("=").append(x.second);
      query_pos_found = true;
    } else {
      url_.append("&").append(x.first).append("=").append(x.second);
    }
  }
  if (inst_) {
    curl_easy_setopt(inst_, CURLOPT_URL, url_.c_str());
  }
}

void CurlRequest::AddHeaders(std::map<std::string, std::string> headers) {
  if (inst_) {
    for (auto const& x : headers) {
      headers_ =
          curl_slist_append(headers_, (x.first + ": " + x.second).c_str());
    }
    curl_easy_setopt(inst_, CURLOPT_HTTPHEADER, headers_);
  }
}

void CurlRequest::AddFile(const std::string name, const std::string filepath) {
  curl_mimepart* part = NULL;
  part = curl_mime_addpart(form_);
  curl_mime_name(part, "file");
  curl_mime_filename(part, name.c_str());
  curl_mime_filedata(part, filepath.c_str());
  is_file_added_ = true;
}

void CurlRequest::AddFileFromString(const std::string name,
                                    const std::string filedata) {
  curl_mimepart* part = NULL;
  part = curl_mime_addpart(form_);
  curl_mime_name(part, "file");
  curl_mime_filename(part, name.c_str());
  curl_mime_data(part, filedata.c_str(), filedata.length());
  is_file_added_ = true;
}

bool CurlRequest::SetEncodedCookie(const std::string key, const std::string val,
                                   const std::string path,
                                   const std::string domain,
                                   const int max_age) {
  int orig_size;
  const char* orig_val_c =
      curl_easy_unescape(inst_, val.c_str(), val.size(), &orig_size);
  const std::string orig_val(orig_val_c);
  return SetCookie(key, val, path, domain, max_age);
}

bool CurlRequest::SetCookie(const std::string key, const std::string val,
                            const std::string path, const std::string domain,
                            const int max_age) {
  std::string cookie;
  cookie.append("Set-Cookie: ").append(key).append("=").append(val).append(";");
  if (path != "") {
    cookie.append(" Path=").append(path).append(";");
  }
  if (domain != "") {
    cookie.append(" Domain=").append(domain).append(";");
  }
  cookie.append(" Max-Age=").append(std::to_string(max_age)).append(";");
  CURLcode res = curl_easy_setopt(inst_, CURLOPT_COOKIELIST, cookie.c_str());
  if (res != CURLE_OK) {
    AERROR << "Set cookie failed:" << curl_easy_strerror(res);
    return false;
  }
  return true;
}

void CurlRequest::SetCookieFile(const std::string filepath) {
  if (inst_) {
    curl_easy_setopt(inst_, CURLOPT_COOKIEFILE, filepath.c_str());
  }
}

void CurlRequest::SetCookieJar(const std::string filepath) {
  if (inst_) {
    curl_easy_setopt(inst_, CURLOPT_COOKIEJAR, filepath.c_str());
  }
}

void CurlRequest::SetVerbose(long flag) {
  if (inst_) {
    curl_easy_setopt(inst_, CURLOPT_VERBOSE, flag);
  }
}

void CurlRequest::SetSslVerifyHost(long flag) {
  if (inst_) {
    curl_easy_setopt(inst_, CURLOPT_SSL_VERIFYHOST, flag);
  }
}

void CurlRequest::SetSslVerifyPeer(long flag) {
  if (inst_) {
    curl_easy_setopt(inst_, CURLOPT_SSL_VERIFYPEER, flag);
  }
}

void CurlRequest::SetUpload(long flag) {
  if (inst_) {
    curl_easy_setopt(inst_, CURLOPT_UPLOAD, flag);
  }
}

void CurlRequest::SetBody(const std::string body) {
  body_ = body;
  body_read_pos_ = 0;
  is_body_setted_ = true;
}

size_t CurlRequest::WriteResponse(const char* ptr, size_t size, size_t nmemb) {
  response_.append(ptr, size * nmemb);
  AINFO << "is_aborted_:" << is_aborted_;
  AINFO << "handle size:" << size * nmemb;
  if (is_aborted_) {
    return 0;
  } else {
    return size * nmemb;
  }
}

size_t CurlRequest::WriteResponseHeader(const char* ptr, size_t size,
                                        size_t nitems) {
  response_headers_buf_.append(ptr, size * nitems);
  return size * nitems;
}

size_t CurlRequest::ReadDataFromString(char* ptr, size_t size, size_t nmemb) {
  size_t max = size * nmemb;
  if (max < 1) {
    return 0;
  }
  size_t remain = body_.length() - body_read_pos_;
  if (remain > 0) {
    size_t copylen = max;
    if (copylen > remain) {
      copylen = remain;
    }
    memcpy(ptr, body_.c_str() + body_read_pos_, copylen);
    body_read_pos_ += copylen;
    return copylen;
  }
  return 0;
}

int CurlRequest::WriteProgress(double dltotal, double dlnow, double ultotal,
                               double ulnow) {
  dltotal_ = dltotal;
  dlnow_ = dlnow;
  ultotal_ = ultotal;
  ulnow_ = ulnow;
  AINFO << "dltotal: " << dltotal << "dlnow: " << dlnow
        << "ultotal: " << ultotal << "ulnow: " << ulnow;
  AINFO << "progress debug:, is_aborted_: " << is_aborted_;
  if (is_aborted_) {
    return 1;
  }
  return 0;
}

void CurlRequest::Abort() { is_aborted_ = true; }

void CurlRequest::SetCallback(std::function<void(int)> callback) {
  callback_ = callback;
}

void CurlRequest::Async() {
  if (!is_async_) {
    is_async_ = true;
    async_running_ = 1;
    // curl_multi_add_handle(multi_handle_, inst_);
  }
}

bool CurlRequest::perform(std::function<void(int)> callback) {
  SetCallback(callback);
  return perform();
}

bool CurlRequest::perform() {
  if (inst_) {
    response_.clear();
    curl_easy_setopt(inst_, CURLOPT_WRITEFUNCTION, CaptureCURLResult);
    curl_easy_setopt(inst_, CURLOPT_WRITEDATA, this);
    response_headers_buf_.clear();
    curl_easy_setopt(inst_, CURLOPT_HEADERFUNCTION, CaptureCURLHeader);
    curl_easy_setopt(inst_, CURLOPT_HEADERDATA, this);
    if (is_body_setted_) {
      SetUpload(1L);
      curl_easy_setopt(inst_, CURLOPT_READFUNCTION, SendBodyFromString);
      curl_easy_setopt(inst_, CURLOPT_READDATA, this);
      curl_easy_setopt(inst_, CURLOPT_INFILESIZE_LARGE,
                       (curl_off_t)body_.length());
    }
    if (is_file_added_) {
      curl_easy_setopt(inst_, CURLOPT_MIMEPOST, form_);
    }
    dltotal_ = 0;
    dlnow_ = 0;
    ultotal_ = 0;
    ulnow_ = 0;
    curl_easy_setopt(inst_, CURLOPT_PROGRESSFUNCTION, CaptureCURLProgress);
    curl_easy_setopt(inst_, CURLOPT_PROGRESSDATA, this);
    if (is_async_) {
      async_polling_thread_ = cyber::Async(&CurlRequest::AsyncPerform, this);
    } else {
      CURLcode res = curl_easy_perform(inst_);
      if (res != CURLE_OK) {
        if (is_aborted_) {
          AWARN << "CURL " << url_ << " Aborted";
          return true;
        } else {
          AERROR << "CURL " << url_ << " Failed " << curl_easy_strerror(res);
          return false;
        }
      }
      ProcessResponseHeaders();
    }
    return true;
  }
  return false;
}

const std::string CurlRequest::Response() { return response_; }

const std::map<std::string, std::string> CurlRequest::ResponseHeaders() {
  return response_headers_;
}

const std::string CurlRequest::ResponseHeader(const std::string key) {
  auto iter = response_headers_.find(key);
  if (iter == response_headers_.end()) {
    return "";
  }
  return iter->second;
}

void CurlRequest::AsyncPerform() {
  CURLcode res = curl_easy_perform(inst_);
  if (res != CURLE_OK) {
    AERROR << "CURL " << url_ << " Failed " << curl_easy_strerror(res);
    return;
  }
  ProcessResponseHeaders();
  if (!is_aborted_) {
    callback_(res);
  }
}

void CurlRequest::ProcessResponseHeaders() {
  response_headers_.clear();
  std::vector<std::string> header_lines;
  std::string sp = "\r\n";
  std::string line;
  std::string ss = response_headers_buf_;
  int line_cnt = 0;
  for (size_t pos = ss.find(sp); pos != std::string::npos; pos = ss.find(sp)) {
    line = ss.substr(0, pos);
    if (line_cnt) {
      size_t kv_pos = line.find(": ");
      if (kv_pos != std::string::npos) {
        response_headers_[line.substr(0, kv_pos)] = line.substr(kv_pos + 2);
      } else {
        response_headers_[line] = "";
      }
    }
    ss.erase(0, pos + sp.length());
    line_cnt++;
  }
}

}  // namespace dreamview
}  // namespace apollo
