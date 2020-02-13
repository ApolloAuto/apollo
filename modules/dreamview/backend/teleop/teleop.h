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

#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>

#include "cyber/cyber.h"
#include "third_party/json/json.hpp"

#ifdef TELEOP
#include "modules/planning/proto/pad_msg.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/teleop/daemon/proto/daemon_cmd.pb.h"
#include "modules/teleop/daemon/proto/daemon_rpt.pb.h"
#include "modules/teleop/modem/proto/modem_info.pb.h"
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
  // send a command to the remote daemon to start or stop
  // video encoders and voip encoders
  void SendAudioStreamCmd(bool start_stop);
  void SendMicStreamCmd(bool start_stop);
  void SendVideoStreamCmd(bool start_stop);
  // planner commands
  void SendEstopCmd();
  void SendPullOverCmd();
  void SendResumeCruiseCmd();

  void UpdateModemInfo(
      const std::shared_ptr<modules::teleop::modem::ModemInfo> &modem_info);
#endif

  std::unique_ptr<cyber::Node> node_;

  WebSocketHandler *websocket_;

#ifdef TELEOP
  // modem info readers and callback
  std::shared_ptr<cyber::Reader<modules::teleop::modem::ModemInfo>>
      modem0_info_reader_;
  std::shared_ptr<cyber::Reader<modules::teleop::modem::ModemInfo>>
      modem1_info_reader_;
  std::shared_ptr<cyber::Reader<modules::teleop::modem::ModemInfo>>
      modem2_info_reader_;
  // modem info callback
  void UpdateModem(
      const std::string &modem_id,
      const std::shared_ptr<modules::teleop::modem::ModemInfo> &modem_info);
  // planning message reader
  std::shared_ptr<cyber::Reader<apollo::planning::ADCTrajectory>>
      planning_reader_;

  // daemon report readers and callback
  void UpdateCarDaemonRpt(
      const std::shared_ptr<modules::teleop::daemon::DaemonRpt> &rpt);
  void UpdateOperatorDaemonRpt(
      const std::shared_ptr<modules::teleop::daemon::DaemonRpt> &rpt);
  std::shared_ptr<cyber::Reader<modules::teleop::daemon::DaemonRpt>>
      remote_daemon_rpt_reader_;
  std::shared_ptr<cyber::Reader<modules::teleop::daemon::DaemonRpt>>
      local_daemon_rpt_reader_;
  // daemon commands writers
  std::shared_ptr<cyber::Writer<modules::teleop::daemon::DaemonCmd>>
      remote_daemon_cmd_writer_;
  std::shared_ptr<cyber::Writer<modules::teleop::daemon::DaemonCmd>>
      local_daemon_cmd_writer_;

  // planning driving actions  and feedback
  std::shared_ptr<cyber::Writer<apollo::planning::PadMessage>>
      pad_message_writer_;
  void UpdatePlanning(
      const std::shared_ptr<apollo::planning::ADCTrajectory> &msg);
#endif

  // Store teleop status
  nlohmann::json teleop_status_;

  // Mutex to protect concurrent access to teleop_status_.
  // NOTE: Use boost until we upgrade to std version with rwlock support.
  boost::shared_mutex mutex_;
};

}  // namespace dreamview
}  // namespace apollo
