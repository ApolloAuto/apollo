/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "boost/thread/locks.hpp"
#include "boost/thread/shared_mutex.hpp"
#include "cyber/cyber.h"
#include "third_party/json/json.hpp"

#ifdef TELEOP
#include "modules/car1/network/proto/modem_info.pb.h"
#include "modules/car1/teleop/proto/daemon_service_cmd.pb.h"
#endif

#include "modules/dreamview/backend/handlers/websocket_handler.h"

namespace apollo {
namespace dreamview {

class TeleopService {
 public:
  TeleopService(WebSocketHandler *websocket);

  void Start();

 private:
  void RegisterMessageHandlers();
  void SendStatus(WebSocketHandler::Connection *conn);

#ifdef TELEOP
  void UpdateModemInfo(
      const std::shared_ptr<modules::car1::network::ModemInfo> &modem_info);
#endif

  std::unique_ptr<cyber::Node> node_;

  WebSocketHandler *websocket_;

#ifdef TELEOP
  std::shared_ptr<cyber::Reader<modules::car1::network::ModemInfo>>
      modem_info_reader_;

  std::shared_ptr<cyber::Writer<modules::car1::teleop::DaemonServiceCmd>>
      daemon_cmd_writer_;
#endif

  // Store teleop status
  nlohmann::json teleop_status_;

  // Mutex to protect concurrent access to teleop_status_.
  // NOTE: Use boost until we upgrade to std version with rwlock support.
  boost::shared_mutex mutex_;
};

}  // namespace dreamview
}  // namespace apollo
