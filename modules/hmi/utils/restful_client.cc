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

#include "modules/hmi/utils/restful_client.h"

#include <curlpp/Easy.hpp>
#include <curlpp/Exception.hpp>
#include <curlpp/Options.hpp>
#include <curlpp/cURLpp.hpp>

#include "glog/logging.h"
#include "google/protobuf/util/json_util.h"
#include "modules/common/log.h"

namespace apollo {
namespace hmi {

RestfulClient::STATUS RestfulClient::Post(
    const google::protobuf::Message& proto) {
  std::string json;
  const auto status = google::protobuf::util::MessageToJsonString(proto, &json);
  CHECK(status.ok()) << status.error_message();
  AINFO << "Put proto message to " << url_ << ":\n" << proto.DebugString();

  try {
    curlpp::Cleanup cleaner;
    curlpp::Easy request;

    request.setOpt(new curlpp::options::Url(url_));
    request.setOpt(
        new curlpp::options::HttpHeader({"Content-Type: application/json"}));
    request.setOpt(new curlpp::options::PostFields(json));
    request.setOpt(new curlpp::options::PostFieldSize(json.length()));
    request.perform();
  } catch (curlpp::LogicError& e) {
    AERROR << "LogicError: " << e.what();
    return LOGIC_ERROR;
  } catch (curlpp::RuntimeError& e) {
    AERROR << "RuntimeError: " << e.what();
    return RUNTIME_ERROR;
  }

  return SUCCESS;
}

}  // namespace hmi
}  // namespace apollo
