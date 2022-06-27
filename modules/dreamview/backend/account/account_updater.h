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

#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>

#include "nlohmann/json.hpp"

#include "modules/dreamview/proto/account_info.pb.h"

#include "cyber/cyber.h"
#include "modules/dreamview/backend/account/login_callback_handler.h"
#include "modules/dreamview/backend/handlers/websocket_handler.h"

/**
 * @namespace apollo::dreamview
 * @brief apollo::dreamview
 */
namespace apollo {
namespace dreamview {

/**
 * @class AccountUpdater
 * @brief WebSockerHandler wraper to communicate to frontend for account info
 * updating, logging in and logging out
 */
class AccountUpdater {
 public:
  /**
   * @brief Constructor with websocket handler
   * @param websocket Pointer of websocket handler that has been attached to
   * the server.
   * @param login_callback_handler Pointer of handler
   */
  AccountUpdater(WebSocketHandler* websocket,
                 LoginCallbackHandler* login_callback_handler);

  ~AccountUpdater();

  /**
   * @breif Start
   */
  void Start();
  void Stop();

 private:
  std::string GetCacheDir();
  std::string GetSessionFilePath();
  std::string GetCookiesFilePath();
  void LoadSession();
  void SaveSession();
  void DeleteCookies();
  void UpdateInfoBySid(const std::string sid);
  void RegisterMessageHandlers();

  /**
   * @breif Fetch account info
   */
  void FetchAccountInfo(const std::string sid);
  AccountInfo account_info_;
  std::string sid_;
  std::unique_ptr<cyber::Node> node_;
  std::shared_ptr<cyber::Writer<AccountInfo>> account_info_writer_;
  WebSocketHandler* websocket_;
  LoginCallbackHandler* login_callback_handler_;

  // Mutex to protect concurrent access to teleop_status_.
  // NOTE: Use boost until we upgrade to std version with rwlock support.
  boost::shared_mutex mutex_;
};

}  // namespace dreamview
}  // namespace apollo
