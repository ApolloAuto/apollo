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
 * @file login_callback_handler.h
 */

#pragma once

#include "CivetServer.h"

#include "cyber/cyber.h"

/**
 * @namespace apollo::dreamview
 * @brief apolo::dreamview
 */
namespace apollo {
namespace dreamview {

/**
 * @class LoginCallbackHandlero
 *
 * @brief The login callback handler, save login session
 */
class LoginCallbackHandler : public CivetHandler {
 public:
  LoginCallbackHandler();

  bool handleGet(CivetServer* server, struct mg_connection* conn);

  using LoginSuccessHandler = std::function<void(const std::string sid)>;
  inline void RegisterLoginSuccessHandler(LoginSuccessHandler handler) {
    login_success_handlers_.push_back(handler);
  }

 private:
  std::vector<LoginSuccessHandler> login_success_handlers_;

  std::mutex mutex_;
  std::condition_variable cvar_;
};

}  // namespace dreamview
}  // namespace apollo
