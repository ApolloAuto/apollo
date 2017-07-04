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

/**
 * @file restful_client.h
 * @brief the class of RestfulClient
 */

#ifndef MODULES_HMI_UTILS_RESTFUL_CLIENT_H_
#define MODULES_HMI_UTILS_RESTFUL_CLIENT_H_

#include <google/protobuf/message.h>
#include <string>

/**
 * @namespace apollo::hmi
 * @brief apollo::hmi
 */
namespace apollo {
namespace hmi {

/**
 * @class RestfulClient
 *
 * @brief Client to send request to restful APIs. RestfulClient class provides
 * two public functions, which support the construction of client with an APT
 * url and posting a proto to target API.
 */
class RestfulClient {
 public:
  enum STATUS {
    SUCCESS,
    LOGIC_ERROR,
    RUNTIME_ERROR,
  };

  /*
   * @brief constructor, used explicit keyword to avoid implicit construction.
   * It init client with an APT url
   * @param url the API url string.
   */
  explicit RestfulClient(const std::string& url) : url_(url) {}

  /*
   * @brief post a proto to target API. Note that the data is transferred as
   * JSON.
   * @param proto the proto to be posted to target API.
   * @return the status define by google::protobuf::util::MessageToJsonString
   */
  STATUS Post(const google::protobuf::Message& proto);

 private:
  const std::string url_;
};

}  // namespace hmi
}  // namespace apollo

#endif  // MODULES_HMI_UTILS_RESTFUL_CLIENT_H_
