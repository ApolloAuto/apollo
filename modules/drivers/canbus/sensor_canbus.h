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
 * @file
 */

#pragma once

#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "cyber/common/file.h"
#include "cyber/component/component.h"

#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/time/time.h"
#include "modules/common/util/util.h"
#include "modules/drivers/canbus/can_client/can_client.h"
#include "modules/drivers/canbus/can_client/can_client_factory.h"
#include "modules/drivers/canbus/can_comm/can_receiver.h"
#include "modules/drivers/canbus/can_comm/message_manager.h"
#include "modules/drivers/canbus/proto/can_card_parameter.pb.h"
#include "modules/drivers/canbus/proto/sensor_canbus_conf.pb.h"
#include "modules/drivers/canbus/sensor_gflags.h"

/**
 * @namespace apollo::drivers
 * @brief apollo::drivers
 */
namespace apollo {
namespace drivers {

/**
 * @class SensorCanbus
 *
 * @brief template of canbus-based sensor module main class (e.g., mobileye).
 */

using apollo::common::ErrorCode;
// using apollo::common::Status;
using apollo::common::monitor::MonitorMessageItem;
using apollo::common::time::Clock;
using apollo::drivers::canbus::CanClient;
using apollo::drivers::canbus::CanClientFactory;
using apollo::drivers::canbus::CanReceiver;
using apollo::drivers::canbus::SensorCanbusConf;
template <typename T>
using Writer = apollo::cyber::Writer<T>;

template <typename SensorType>
class SensorCanbus : public apollo::cyber::Component<> {
 public:
  // TODO(lizh): check whether we need a new msg item, say
  // MonitorMessageItem::SENSORCANBUS
  SensorCanbus()
      : monitor_logger_buffer_(
            apollo::common::monitor::MonitorMessageItem::CANBUS) {}
  ~SensorCanbus();

  /**
   * @brief module initialization function
   * @return initialization status
   */
  bool Init() override;

 private:
  bool Start();
  void PublishSensorData();
  void OnTimer();
  void DataTrigger();
  bool OnError(const std::string &error_msg);
  void RegisterCanClients();

  SensorCanbusConf canbus_conf_;
  std::unique_ptr<CanClient> can_client_;
  CanReceiver<SensorType> can_receiver_;
  std::unique_ptr<canbus::MessageManager<SensorType>> sensor_message_manager_;
  std::unique_ptr<std::thread> thread_;

  int64_t last_timestamp_ = 0;
  std::unique_ptr<cyber::Timer> timer_;
  common::monitor::MonitorLogBuffer monitor_logger_buffer_;
  std::mutex mutex_;
  volatile bool data_trigger_running_ = false;
  std::shared_ptr<Writer<SensorType>> sensor_writer_;
};

// method implementations

template <typename SensorType>
bool SensorCanbus<SensorType>::Init() {
  // load conf
  if (!cyber::common::GetProtoFromFile(config_file_path_, &canbus_conf_)) {
    return OnError("Unable to load canbus conf file: " + config_file_path_);
  }

  AINFO << "The canbus conf file is loaded: " << config_file_path_;
  ADEBUG << "Canbus_conf:" << canbus_conf_.ShortDebugString();

  // Init can client
  auto *can_factory = CanClientFactory::Instance();
  can_factory->RegisterCanClients();
  can_client_ = can_factory->CreateCANClient(canbus_conf_.can_card_parameter());
  if (!can_client_) {
    return OnError("Failed to create can client.");
  }
  AINFO << "Can client is successfully created.";

  sensor_message_manager_.reset(new canbus::MessageManager<SensorType>());
  if (sensor_message_manager_ == nullptr) {
    return OnError("Failed to create message manager.");
  }
  AINFO << "Sensor message manager is successfully created.";

  if (can_receiver_.Init(can_client_.get(), sensor_message_manager_.get(),
                         canbus_conf_.enable_receiver_log()) != ErrorCode::OK) {
    return OnError("Failed to init can receiver.");
  }
  AINFO << "The can receiver is successfully initialized.";
  sensor_writer_ = node_->CreateWriter<SensorType>(FLAGS_sensor_node_name);
  return Start();
}

template <typename SensorType>
bool SensorCanbus<SensorType>::Start() {
  // 1. init and start the can card hardware
  if (can_client_->Start() != ErrorCode::OK) {
    return OnError("Failed to start can client");
  }
  AINFO << "Can client is started.";

  // 2. start receive first then send
  if (can_receiver_.Start() != ErrorCode::OK) {
    return OnError("Failed to start can receiver.");
  }
  AINFO << "Can receiver is started.";

  // 3. set timer to trigger publish info periodically
  // if sensor_freq == 0, then it is event-triggered publishment.
  // no need for timer.
  if (FLAGS_sensor_freq > 0) {
    double duration_ms = 1000.0 / FLAGS_sensor_freq;
    timer_.reset(new cyber::Timer(static_cast<uint32_t>(duration_ms),
                                  [this]() { this->OnTimer(); }, false));
    timer_->Start();
  } else {
    data_trigger_running_ = true;
    thread_.reset(new std::thread([this] { DataTrigger(); }));
    if (thread_ == nullptr) {
      AERROR << "Unable to create data trigger thread.";
      return OnError("Failed to start data trigger thread.");
    }
  }

  // last step: publish monitor messages
  monitor_logger_buffer_.INFO("Canbus is started.");

  return true;
}

template <typename SensorType>
void SensorCanbus<SensorType>::OnTimer() {
  PublishSensorData();
}

template <typename SensorType>
void SensorCanbus<SensorType>::DataTrigger() {
  std::condition_variable *cvar = sensor_message_manager_->GetMutableCVar();
  while (data_trigger_running_) {
    std::unique_lock<std::mutex> lock(mutex_);
    cvar->wait(lock);
    PublishSensorData();
    sensor_message_manager_->ClearSensorData();
  }
}

template <typename SensorType>
SensorCanbus<SensorType>::~SensorCanbus() {
  if (FLAGS_sensor_freq > 0) {
    timer_->Stop();
  }

  can_receiver_.Stop();
  can_client_->Stop();

  if (data_trigger_running_) {
    data_trigger_running_ = false;
    if (thread_ != nullptr && thread_->joinable()) {
      sensor_message_manager_->GetMutableCVar()->notify_all();
      thread_->join();
    }
    thread_.reset();
  }
  AINFO << "Data trigger stopped [ok].";
}

// Send the error to monitor and return it
template <typename SensorType>
bool SensorCanbus<SensorType>::OnError(const std::string &error_msg) {
  monitor_logger_buffer_.ERROR(error_msg);
  return false;
}

}  // namespace drivers
}  // namespace apollo
