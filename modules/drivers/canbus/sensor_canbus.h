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

#ifndef MODULES_DRIVERS_CANBUS_SENSOR_CANBUS_H_
#define MODULES_DRIVERS_CANBUS_SENSOR_CANBUS_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <mutex>
#include <thread>
#include <condition_variable>

#include "ros/include/ros/ros.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/adapters/proto/adapter_config.pb.h"
#include "modules/common/apollo_app.h"
#include "modules/common/macro.h"
#include "modules/common/monitor/monitor.h"
#include "modules/common/time/time.h"
#include "modules/common/util/util.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/drivers/canbus/can_client/can_client.h"
#include "modules/drivers/canbus/can_client/can_client_factory.h"
#include "modules/drivers/canbus/can_comm/can_receiver.h"
#include "modules/drivers/canbus/can_comm/message_manager.h"
#include "modules/drivers/canbus/sensor_gflags.h"
#include "modules/drivers/canbus/proto/can_card_parameter.pb.h"
#include "modules/drivers/canbus/proto/sensor_canbus_conf.pb.h"
#include "modules/hmi/utils/hmi_status_helper.h"

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

using apollo::common::adapter::AdapterConfig;
using apollo::common::adapter::AdapterManager;
using apollo::common::monitor::MonitorMessageItem;
using apollo::common::Status;
using apollo::common::ErrorCode;
using apollo::common::time::Clock;
using apollo::drivers::canbus::CanClientFactory;
using apollo::drivers::canbus::CanClient;
using apollo::drivers::canbus::CanReceiver;
using apollo::drivers::canbus::SensorCanbusConf;

template <typename SensorType>
class SensorCanbus : public apollo::common::ApolloApp {
 public:
  // TODO(lizh): check whether we need a new msg item, say
  // MonitorMessageItem::SENSORCANBUS
  SensorCanbus()
      : monitor_(apollo::common::monitor::MonitorMessageItem::CANBUS) {}

  /**
  * @brief obtain module name
  * @return module name
  */
  std::string Name() const override;

  /**
  * @brief module initialization function
  * @return initialization status
  */
  apollo::common::Status Init() override;

  /**
  * @brief module start function
  * @return start status
  */
  apollo::common::Status Start() override;

  /**
  * @brief module stop function
  */
  void Stop() override;

 private:
  void PublishSensorData();
  void OnTimer(const ros::TimerEvent &event);
  void DataTrigger();
  apollo::common::Status OnError(const std::string &error_msg);
  void RegisterCanClients();

  SensorCanbusConf canbus_conf_;
  std::unique_ptr<CanClient> can_client_;
  CanReceiver<SensorType> can_receiver_;
  std::unique_ptr<canbus::MessageManager<SensorType>> sensor_message_manager_;
  std::unique_ptr<std::thread> thread_;

  int64_t last_timestamp_ = 0;
  ros::Timer timer_;
  apollo::common::monitor::Monitor monitor_;
  std::mutex mutex_;
  bool data_trigger_running_ = false;
};

// method implementations

template <typename SensorType>
std::string SensorCanbus<SensorType>::Name() const {
  return FLAGS_hmi_name;
}

template <typename SensorType>
Status SensorCanbus<SensorType>::Init() {
  // load conf
  if (!::apollo::common::util::GetProtoFromFile(FLAGS_sensor_conf_file,
                                                &canbus_conf_)) {
    return OnError("Unable to load canbus conf file: " +
                   FLAGS_sensor_conf_file);
  }

  AINFO << "The canbus conf file is loaded: " << FLAGS_sensor_conf_file;
  ADEBUG << "Canbus_conf:" << canbus_conf_.ShortDebugString();

  // Init can client
  auto *can_factory = CanClientFactory::instance();
  can_factory->RegisterCanClients();
  can_client_ = can_factory->CreateCANClient(canbus_conf_.can_card_parameter());
  if (!can_client_) {
    return OnError("Failed to create can client.");
  }
  AINFO << "Can client is successfully created.";

  sensor_message_manager_ = std::unique_ptr<canbus::MessageManager<SensorType>>(
      new canbus::MessageManager<SensorType>());
  if (sensor_message_manager_ == nullptr) {
    return OnError("Failed to create message manager.");
  }
  AINFO << "Sensor message manager is successfully created.";

  if (can_receiver_.Init(can_client_.get(), sensor_message_manager_.get(),
                         canbus_conf_.enable_receiver_log()) != ErrorCode::OK) {
    return OnError("Failed to init can receiver.");
  }
  AINFO << "The can receiver is successfully initialized.";

  AdapterManager::Init(FLAGS_adapter_config_filename);

  AINFO << "The adapter manager is successfully initialized.";

  return Status::OK();
}

template <typename SensorType>
Status SensorCanbus<SensorType>::Start() {
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
    const double duration = 1.0 / FLAGS_sensor_freq;
    timer_ = AdapterManager::CreateTimer(
        ros::Duration(duration), &SensorCanbus<SensorType>::OnTimer, this);
  } else {
    data_trigger_running_ = true;
    thread_.reset(new std::thread([this] { DataTrigger(); }));
    if (thread_ == nullptr) {
      AERROR << "Unable to create data trigger thread.";
      return OnError("Failed to start data trigger thread.");
    }
  }

  // last step: publish monitor messages
  apollo::common::monitor::MonitorBuffer buffer(&monitor_);
  buffer.INFO("Canbus is started.");

  return Status::OK();
}

template <typename SensorType>
void SensorCanbus<SensorType>::OnTimer(const ros::TimerEvent &) {
  PublishSensorData();
}

template <typename SensorType>
void SensorCanbus<SensorType>::DataTrigger() {
  std::condition_variable* cvar = sensor_message_manager_->GetMutableCVar();
  while (data_trigger_running_) {
    std::unique_lock<std::mutex> lock(mutex_);
    cvar->wait(lock);
    //TODO: this is a log for test.  Please remove it after onboard test.
    AINFO << "===== Pulibsh Sensor Data =====";
    PublishSensorData();
    sensor_message_manager_->ClearSensorData();
  }
}

template <typename SensorType>
void SensorCanbus<SensorType>::Stop() {
  timer_.stop();

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
Status SensorCanbus<SensorType>::OnError(const std::string &error_msg) {
  apollo::common::monitor::MonitorBuffer buffer(&monitor_);
  buffer.ERROR(error_msg);
  return Status(ErrorCode::CANBUS_ERROR, error_msg);
}

}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_CANBUS_SENSOR_CANBUS_H_
