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

/**
 * @file client_wrapper.h
 **/

#pragma once

#include <string>
#include <memory>

#include "cyber/service/client.h"

/**
 * @namespace apollo::common
 * @brief apollo::common
 */
namespace apollo {
namespace common {

/**
 * @brief Wrapper of cyber::Client which sends a topic with service name when
 * SendRequst is invoked.
 */
template <typename Request, typename Response>
class ClientWrapper {
 public:
  ClientWrapper(const std::shared_ptr<cyber::Node>& node,
                const std::string& service_name);
  /**
   * @brief Request the Service with a shared ptr Request type
   *
   * @param request shared ptr of Request type
   * @param timeout_s request timeout, if timeout, response will be empty
   * @return std::shared_ptr<Response> result of this request
   */
  std::shared_ptr<Response> SendRequest(
      std::shared_ptr<Request> request,
      const std::chrono::seconds& timeout_s = std::chrono::seconds(5));

  /**
   * @brief Request the Service with a Request object
   *
   * @param request Request object
   * @param timeout_s request timeout, if timeout, response will be empty
   * @return std::shared_ptr<Response> result of this request
   */
  std::shared_ptr<Response> SendRequest(
      const Request& request,
      const std::chrono::seconds& timeout_s = std::chrono::seconds(5));

  /**
   * @brief Send Request shared ptr asynchronously
   */
  std::shared_future<Response> AsyncSendRequest(
      std::shared_ptr<Request> request);

  /**
   * @brief Send Request object asynchronously
   */
  std::shared_future<Response> AsyncSendRequest(const Request& request);

  /**
   * @brief Send Request shared ptr asynchronously and invoke `cb` after we get
   * response
   *
   * @param request Request shared ptr
   * @param cb callback function after we get response
   * @return std::shared_future<Response> a `std::future`
   * shared ptr
   */
  std::shared_future<Response> AsyncSendRequest(
      std::shared_ptr<Request> request,
      std::function<void(std::shared_future<std::shared_ptr<Response>>)>&& cb);

 private:
  std::shared_ptr<apollo::cyber::Client<Request, Response>> client_;
  std::shared_ptr<apollo::cyber::Writer<Request>> request_writer_;
};

template <typename Request, typename Response>
ClientWrapper<Request, Response>::ClientWrapper(
    const std::shared_ptr<cyber::Node>& node, const std::string& service_name)
    : client_(node->CreateClient<Request, Response>(service_name)),
      request_writer_(node->CreateWriter<Request>(service_name)) {}

template <typename Request, typename Response>
std::shared_ptr<Response> ClientWrapper<Request, Response>::SendRequest(
    std::shared_ptr<Request> request, const std::chrono::seconds& timeout_s) {
  auto response = client_->SendRequest(request);
  request_writer_->Write(request);
  return response;
}

template <typename Request, typename Response>
std::shared_ptr<Response> ClientWrapper<Request, Response>::SendRequest(
    const Request& request, const std::chrono::seconds& timeout_s) {
  auto response = client_->SendRequest(request);
  request_writer_->Write(request);
  return response;
}

template <typename Request, typename Response>
std::shared_future<Response> ClientWrapper<Request, Response>::AsyncSendRequest(
    std::shared_ptr<Request> request) {
  auto response = client_->AsyncSendRequest(request);
  request_writer_->Write(request);
  return response;
}

template <typename Request, typename Response>
std::shared_future<Response> ClientWrapper<Request, Response>::AsyncSendRequest(
    const Request& request) {
  auto response = client_->AsyncSendRequest(request);
  request_writer_->Write(request);
  return response;
}

template <typename Request, typename Response>
std::shared_future<Response> ClientWrapper<Request, Response>::AsyncSendRequest(
    std::shared_ptr<Request> request,
    std::function<void(std::shared_future<std::shared_ptr<Response>>)>&& cb) {
  auto response = client_->AsyncSendRequest(request, cb);
  request_writer_->Write(request);
  return response;
}

}  // namespace common
}  // namespace apollo
