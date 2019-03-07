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

#include "modules/common/util/http_client.h"

#include <curlpp/Easy.hpp>
#include <curlpp/Exception.hpp>
#include <curlpp/Options.hpp>
#include <curlpp/cURLpp.hpp>

#include "cyber/common/log.h"
#include "modules/common/util/string_util.h"

namespace apollo {
namespace common {
namespace util {

using Json = nlohmann::json;

Status HttpClient::Post(const std::string &url, const Json &json,
                        std::string *result) {
  try {
    curlpp::Cleanup cleaner;
    curlpp::Easy request;
    std::ostringstream response;

    request.setOpt(new curlpp::options::Url(url));
    request.setOpt(
        new curlpp::options::HttpHeader({"Content-Type: application/json"}));
    const std::string data = json.dump();
    request.setOpt(new curlpp::options::PostFields(data));
    request.setOpt(new curlpp::options::PostFieldSize(data.length()));
    request.setOpt(new curlpp::options::WriteStream(&response));

    // Perform request and get response string.
    request.perform();
    if (result != nullptr) {
      *result = response.str();
    }
  } catch (curlpp::LogicError &e) {
    AERROR << e.what();
    return Status(ErrorCode::HTTP_LOGIC_ERROR, e.what());
  } catch (curlpp::RuntimeError &e) {
    AERROR << e.what();
    return Status(ErrorCode::HTTP_RUNTIME_ERROR, e.what());
  }

  return Status::OK();
}

Status HttpClient::Post(const std::string &url, const Json &json,
                        Json *result) {
  if (!StartWith(url, "https://")) {
    return Status(ErrorCode::HTTP_LOGIC_ERROR, "Use HTTPS to post data!");
  }
  std::string response;
  const auto status = Post(url, json, &response);
  if (status.ok()) {
    try {
      *result = Json::parse(response.begin(), response.end());
    } catch (...) {
      return Status(ErrorCode::HTTP_RUNTIME_ERROR,
                    "Cannot parse response as json.");
    }
  }
  return status;
}

}  // namespace util
}  // namespace common
}  // namespace apollo
