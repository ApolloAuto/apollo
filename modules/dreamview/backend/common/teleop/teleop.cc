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

#include "modules/dreamview/backend/common/teleop/teleop.h"

#include "cyber/common/log.h"
#include "cyber/time/time.h"
#include "google/protobuf/util/json_util.h"
#include "modules/common/util/message_util.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"

#include <iomanip>
#include <sstream>

namespace apollo {
namespace dreamview {

using Json = nlohmann::json;
using apollo::cyber::Time;
using apollo::planning::ADCTrajectory;
using apollo::planning::DrivingAction;
using apollo::planning::ScenarioConfig;
using ::google::protobuf::util::MessageToJsonString;
using modules::teleop::daemon::DaemonCmd;
using modules::teleop::daemon::DaemonRpt;
using modules::teleop::modem::ModemInfo;
using apollo::external_command::ActionCommand;
using apollo::external_command::CommandStatus;

// modem ids
const std::string modem0_id = "0";
const std::string modem1_id = "1";
const std::string modem2_id = "2";

// number of simultaneous video encoders
static const unsigned int kEncoderCount = 3;
// delay between to consecutive  msg writes
static const unsigned int kWriteWaitMs = 50;

const std::string start_cmd = "start";
const std::string stop_cmd = "kill";

// channels
const std::string modem0_channel = "/apollo/teleop/modem/modem0";
const std::string modem1_channel = "/apollo/teleop/modem/modem1";
const std::string modem2_channel = "/apollo/teleop/modem/modem2";
const std::string remote_daemon_cmd_channel =
    "/apollo/teleop/daemon/remote/cmd";
const std::string remote_daemon_rpt_channel =
    "/apollo/teleop/daemon/remote/rpt";
const std::string local_daemon_cmd_channel = "/apollo/teleop/daemon/local/cmd";
const std::string local_daemon_rpt_channel = "/apollo/teleop/daemon/local/rpt";
const std::string planning_channel = "/apollo/planning";
const std::string planning_pad_channel = "/apollo/planning/pad";

TeleopService::TeleopService(WebSocketHandler *websocket)
    : node_(cyber::CreateNode("teleop")), websocket_(websocket) {
  RegisterMessageHandlers();

  teleop_status_["audio"] = false;
  teleop_status_["audio_starting"] = false;
  teleop_status_["audio_stopping"] = false;
  teleop_status_["mic"] = false;
  teleop_status_["mic_starting"] = false;
  teleop_status_["mic_stopping"] = false;
  teleop_status_["video"] = false;
  teleop_status_["video_starting"] = false;
  teleop_status_["video_stopping"] = false;
  teleop_status_["pulling_over"] = false;
  teleop_status_["e_stopping"] = false;
  teleop_status_["resuming_autonomy"] = false;
}

void TeleopService::Start() {
  // TODO get topic names from proto
  // TODO update proto to get all modems' info combined with rank

  modem0_info_reader_ = node_->CreateReader<ModemInfo>(
      modem0_channel, [this](const std::shared_ptr<ModemInfo> &msg) {
        UpdateModem(modem0_id, msg);
      });

  modem1_info_reader_ = node_->CreateReader<ModemInfo>(
      modem1_channel, [this](const std::shared_ptr<ModemInfo> &msg) {
        UpdateModem(modem1_id, msg);
      });

  modem2_info_reader_ = node_->CreateReader<ModemInfo>(
      modem2_channel, [this](const std::shared_ptr<ModemInfo> &msg) {
        UpdateModem(modem2_id, msg);
      });

  planning_reader_ = node_->CreateReader<ADCTrajectory>(
      planning_channel, [this](const std::shared_ptr<ADCTrajectory> &msg) {
        UpdatePlanning(msg);
      });

  remote_daemon_cmd_writer_ =
      node_->CreateWriter<DaemonCmd>(remote_daemon_cmd_channel);

  local_daemon_cmd_writer_ =
      node_->CreateWriter<DaemonCmd>(local_daemon_cmd_channel);

  remote_daemon_rpt_reader_ = node_->CreateReader<DaemonRpt>(
      remote_daemon_rpt_channel,
      [this](const std::shared_ptr<DaemonRpt> &msg) {
        UpdateCarDaemonRpt(msg);
      });

  local_daemon_rpt_reader_ = node_->CreateReader<DaemonRpt>(
      local_daemon_rpt_channel,
      [this](const std::shared_ptr<DaemonRpt> &msg) {
        UpdateOperatorDaemonRpt(msg);
      });

  action_command_client_ =
      node_->CreateClient<apollo::external_command::ActionCommand,
                          CommandStatus>(FLAGS_action_command_topic);
}

void TeleopService::RegisterMessageHandlers() {
  // Send current teleop status to the new client.
  websocket_->RegisterConnectionReadyHandler(
      [this](WebSocketHandler::Connection *conn) { SendStatus(conn); });
  // Start/Stop local and remote audio
  websocket_->RegisterMessageHandler(
      "ToggleAudio",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        {
          bool start = false;  // false means stop
          // create a scope for the mutex lock
          {
            boost::unique_lock<boost::shared_mutex> writer_lock(mutex_);
            // toggle depending on current state change
            if (teleop_status_["audio_starting"]) {
              teleop_status_["audio_starting"] = false;
              teleop_status_["audio_stopping"] = true;
            } else if (teleop_status_["audio_stopping"]) {
              teleop_status_["audio_starting"] = true;
              teleop_status_["audio_stopping"] = false;
            }
            // not currently starting or stopping video
            else {
              // toggle depending on current state
              if (teleop_status_["audio"]) {
                teleop_status_["audio_stopping"] = true;
                teleop_status_["audio_starting"] = false;
              } else {
                teleop_status_["audio_stopping"] = false;
                teleop_status_["audio_starting"] = true;
              }
            }
            start = teleop_status_["audio_starting"];
          }
          AINFO << "ToggleAudio: " << start;
          SendAudioStreamCmd(start);
        }
      });
  // Mute/Unmute local microphone
  websocket_->RegisterMessageHandler(
      "ToggleMic",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        bool start = false;
        {
          boost::unique_lock<boost::shared_mutex> writer_lock(mutex_);
          // toggle depending on current state change
          if (teleop_status_["mic_starting"]) {
            teleop_status_["mic_starting"] = false;
            teleop_status_["mic_stopping"] = true;
          } else if (teleop_status_["mic_stopping"]) {
            teleop_status_["mic_starting"] = true;
            teleop_status_["mic_stopping"] = false;
          }
          // not currently starting or stopping video
          else {
            // toggle depending on current state
            if (teleop_status_["mic"]) {
              teleop_status_["mic_stopping"] = true;
              teleop_status_["mic_starting"] = false;
            } else {
              teleop_status_["mic_stopping"] = false;
              teleop_status_["mic_starting"] = true;
            }
          }
          start = teleop_status_["mic_starting"];
        }
        AINFO << "ToggleMic: " << start;
        SendMicStreamCmd(start);
      });
  // Start/stop local decoder and viewer, start/stop remote encoder and
  // compositor
  websocket_->RegisterMessageHandler(
      "ToggleVideo",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        bool start = false;  // false means stop
        // create a scope for the mutex lock
        {
          boost::shared_lock<boost::shared_mutex> writer_lock(mutex_);
          // toggle depending on current state change
          if (teleop_status_["video_starting"]) {
            teleop_status_["video_starting"] = false;
            teleop_status_["video_stopping"] = true;
          } else if (teleop_status_["video_stopping"]) {
            teleop_status_["video_starting"] = true;
            teleop_status_["video_stopping"] = false;
          }
          // not currently starting or stopping video
          else {
            // toggle depending on current state
            if (teleop_status_["video"]) {
              teleop_status_["video_stopping"] = true;
              teleop_status_["video_starting"] = false;
            } else {
              teleop_status_["video_stopping"] = false;
              teleop_status_["video_starting"] = true;
            }
          }
          start = teleop_status_["video_starting"];
        }
        // send a start or stop message to the video encoders
        AINFO << "ToggleVideo: " << start;
        SendVideoStreamCmd(start);
      });
  // Issue pull-over command to remote
  websocket_->RegisterMessageHandler(
      "PullOver", [this](const Json &json, WebSocketHandler::Connection *conn) {
        boost::shared_lock<boost::shared_mutex> reader_lock(mutex_);
        SendPullOverCmd();
      });
  // Issue emergency-stop command to remote
  websocket_->RegisterMessageHandler(
      "EStop", [this](const Json &json, WebSocketHandler::Connection *conn) {
        boost::shared_lock<boost::shared_mutex> reader_lock(mutex_);
        SendEstopCmd();
      });
  // Issue resume-cruise command to remote
  websocket_->RegisterMessageHandler(
      "ResumeCruise",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        boost::shared_lock<boost::shared_mutex> reader_lock(mutex_);
        SendResumeCruiseCmd();
      });
  // Request to get updated modem info for client display
  websocket_->RegisterMessageHandler(
      "RequestTeleopStatus",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        SendStatus(conn);
      });
}

void TeleopService::SendStatus(WebSocketHandler::Connection *conn) {
  std::string to_send;
  {
    boost::shared_lock<boost::shared_mutex> reader_lock(mutex_);
    to_send = teleop_status_.dump();
  }
  websocket_->SendData(conn, to_send);
}

void TeleopService::UpdateModem(const std::string &modem_id,
                                const std::shared_ptr<ModemInfo> &modem_info) {
  // TODO simplify data and only send necessary info for display
  // update modem_info_
  if (modem_info->has_technology()) {
    // teleop_status_["modems"][modem_info->provider()] =
    //  modem_info->technology();
    boost::unique_lock<boost::shared_mutex> writer_lock(mutex_);
    std::string str;
    std::stringstream ss(str);
    double rx = 1.0 * modem_info->rx() / (1024 * 1024);
    double tx = 1.0 * modem_info->tx() / (1024 * 1024);

    ss << modem_info->technology();
    ss << std::fixed << std::setw(6) << std::setprecision(2)
       << std::setfill('0');
    ss << " rank: " << modem_info->rank();
    ss << " sig: " << modem_info->signal();
    ss << " q: " << modem_info->quality();
    ss << " rx: " << rx << " MB";
    ss << " tx: " << tx << " MB";
    teleop_status_["modems"][modem_id] = ss.str();
  }
}

// callback for messages that originate from the remote computer
void TeleopService::UpdateCarDaemonRpt(
    const std::shared_ptr<DaemonRpt> &daemon_rpt) {
  {
    bool videoIsRunning = false;
    bool voipIsRunning = false;
    unsigned int runningEncoders = 0;
    for (int i = 0; i < daemon_rpt->services_size(); i++) {
      // look for voip_encoder or encoder0..1.2
      // check 'voip_encoder' first because it contains 'encoder'
      std::string service = daemon_rpt->services(i);
      if (service.find("voip_encoder") != std::string::npos) {
        voipIsRunning = true;
      } else if (service.find("encoder") != std::string::npos) {
        runningEncoders++;
      }
    }

    // all  video encoders are running.
    videoIsRunning = runningEncoders >= kEncoderCount;

    // we may need to write commands to start/stop the video stream
    bool sendStartVideo = false;
    bool sendStopVideo = false;

    bool sendStartAudio = false;
    bool sendStopAudio = false;
    // scope for the lock
    {
      boost::unique_lock<boost::shared_mutex> writer_lock(mutex_);
      teleop_status_["video"] = videoIsRunning;
      teleop_status_["audio"] = voipIsRunning;

      // video currently running
      if (teleop_status_["video"]) {
        if (teleop_status_["video_starting"]) {
          // video has started
          teleop_status_["video_starting"] = false;
        } else if (teleop_status_["video_stopping"]) {
          // not stopped yet
          sendStopVideo = true;
        }
      }
      // video not running
      else {
        if (teleop_status_["video_starting"]) {
          // not started yet
          sendStartVideo = true;
        } else if (teleop_status_["video_stopping"]) {
          // video is stopped
          teleop_status_["video_stopping"] = false;
        }
      }
      // audio currently running
      if (teleop_status_["audio"]) {
        if (teleop_status_["audio_starting"]) {
          // audio has started
          teleop_status_["audio_starting"] = false;
        } else if (teleop_status_["audio_stopping"]) {
          sendStopAudio = true;
        }
      }
      // audio not running
      else {
        if (teleop_status_["audio_starting"]) {
          // not started yet
          sendStartAudio = true;
        } else if (teleop_status_["audio_stopping"]) {
          // video is stopped
          teleop_status_["audio_stopping"] = false;
        }
      }
    }
    if (sendStartVideo || sendStopVideo) {
      SendVideoStreamCmd(sendStartVideo);
    }
    if (sendStartAudio || sendStopAudio) {
      SendAudioStreamCmd(sendStartAudio);
    }
  }
}

// callback for messages that originate from this computer
void TeleopService::UpdateOperatorDaemonRpt(
    const std::shared_ptr<DaemonRpt> &daemon_rpt) {
  {
    bool voipIsRunning = false;
    for (int i = 0; i < daemon_rpt->services_size(); i++) {
      std::string service = daemon_rpt->services(i);
      if (service.find("voip_encoder") != std::string::npos) {
        voipIsRunning = true;
        break;
      }
    }
    bool sendStartMic = false;
    bool sendStopMic = false;
    // scope for the lock
    {
      boost::unique_lock<boost::shared_mutex> writer_lock(mutex_);
      teleop_status_["mic"] = voipIsRunning;
      // mic currently running
      if (teleop_status_["mic"]) {
        if (teleop_status_["mic_starting"]) {
          // mic has started
          teleop_status_["mic_starting"] = false;
        } else if (teleop_status_["mic_stopping"]) {
          sendStopMic = true;
        }
      }
      // mic not running
      else {
        if (teleop_status_["mic_starting"]) {
          // not started yet
          sendStartMic = true;
        } else if (teleop_status_["mic_stopping"]) {
          // video is stopped
          teleop_status_["mic_stopping"] = false;
        }
      }
    }
    if (sendStartMic || sendStopMic) {
      SendMicStreamCmd(sendStartMic);
    }
  }
}

void TeleopService::SendVideoStreamCmd(bool start_stop) {
  DaemonCmd msg;
  if (start_stop) {
    msg.set_cmd(start_cmd);
  } else {
    msg.set_cmd(stop_cmd);
  }
  // we send a message to each encoder.
  for (unsigned int i = 0; i < kEncoderCount; i++) {
    if (i > 0) {
      // delay between sending 2 messages to ensure they are received
      std::this_thread::sleep_for(std::chrono::milliseconds(kWriteWaitMs));
    }
    char encoderName[20];
    snprintf(encoderName, 20, "encoder%u", i);
    msg.set_service(encoderName);
    common::util::FillHeader("dreamview", &msg);
    remote_daemon_cmd_writer_->Write(msg);
    AINFO << encoderName << " " << msg.cmd();
  }
}

void TeleopService::SendAudioStreamCmd(bool start_stop) {
  DaemonCmd msg;
  if (start_stop) {
    msg.set_cmd(start_cmd);
  } else {
    msg.set_cmd(stop_cmd);
  }
  msg.set_service("voip_encoder");
  common::util::FillHeader("dreamview", &msg);
  remote_daemon_cmd_writer_->Write(msg);
  AINFO << "audio " << msg.cmd();
  // audio start / stop implies mic start/stop
  SendMicStreamCmd(start_stop);
}

void TeleopService::SendMicStreamCmd(bool start_stop) {
  // by switching on or off the voip_encoder in the local console
  // we are controlling the mic
  DaemonCmd msg;
  if (start_stop) {
    msg.set_cmd(start_cmd);
  } else {
    msg.set_cmd(stop_cmd);
  }
  msg.set_service("voip_encoder");
  common::util::FillHeader("dreamview", &msg);
  local_daemon_cmd_writer_->Write(msg);
  AINFO << "mic " << msg.cmd();
}

void TeleopService::SendResumeCruiseCmd() {
  AINFO << "Resume cruise";
  auto command = std::make_shared<ActionCommand>();
  command->set_command(apollo::external_command::ActionCommandType::START);
  action_command_client_->SendRequest(command);
}

void TeleopService::SendPullOverCmd() {
  AINFO << "Pull over";
  auto command = std::make_shared<ActionCommand>();
  command->set_command(apollo::external_command::ActionCommandType::PULL_OVER);
  action_command_client_->SendRequest(command);
}

void TeleopService::SendEstopCmd() {
  AINFO << "EStop";
  auto command = std::make_shared<ActionCommand>();
  command->set_command(apollo::external_command::ActionCommandType::STOP);
  action_command_client_->SendRequest(command);
}

void TeleopService::UpdatePlanning(const std::shared_ptr<ADCTrajectory> &msg) {
  static int count = 0;
  ++count;

  if (count % 10 == 0) {
    AINFO << "Update Planning";
  }
  auto scenario_type = msg->debug().planning_data().scenario().scenario_type();

  bool pulled_over = scenario_type == "PULL_OVER";
  bool autonomy_resumed = scenario_type == "PARK_AND_GO";
  bool e_stopped = scenario_type == "EMERGENCY_PULL_OVER";

  bool sendPullOver = false;
  bool sendStop = false;
  bool sendResume = false;
  {
    boost::unique_lock<boost::shared_mutex> writer_lock(mutex_);
    if (pulled_over) {
      if (teleop_status_["pulling_over"]) {
        // pulled over confirmed
        teleop_status_["pulling_over"] = false;
      }
    }
    // not pulled over
    else {
      if (teleop_status_["pulling_over"]) {
        sendPullOver = true;
      }
    }

    if (e_stopped) {
      if (teleop_status_["e_stopping"]) {
        // e stop over confirmed
        teleop_status_["e_stopping"] = false;
      }
    }
    // not e stopped
    else {
      if (teleop_status_["e_stopping"]) {
        sendStop = true;
      }
    }

    if (autonomy_resumed) {
      if (teleop_status_["resuming_autonomy"]) {
        teleop_status_["resuming_autonomy"] = false;
      }
    } else {
      if (teleop_status_["resuming_autonomy"]) {
        sendResume = true;
      }
    }
  }  // writer lock scope

  if (sendResume) {
    SendResumeCruiseCmd();
  }
  if (sendStop) {
    SendEstopCmd();
  }
  if (sendPullOver) {
    SendPullOverCmd();
  }
}

}  // namespace dreamview
}  // namespace apollo
