/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
 * @file lmd_localization.h
 * @brief The class of LMDLocalization.
 */

#ifndef MODULES_LOCALIZATION_LMD_LMD_LOCALIZATION_H_
#define MODULES_LOCALIZATION_LMD_LMD_LOCALIZATION_H_

#include <chrono>
#include <functional>
#include <future>
#include <list>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ros/include/ros/ros.h"

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/drivers/gnss/proto/imu.pb.h"
#include "modules/localization/proto/gps.pb.h"
#include "modules/localization/proto/imu.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"

#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/status/status.h"
#include "modules/localization/lmd/predictor/predictor.h"
#include "modules/localization/localization_base.h"

/**
 * @namespace apollo::localization
 * @brief apollo::localization
 */
namespace apollo {
namespace localization {

/**
 * @class LMDLocalization
 *
 * @brief Generate localization info based on LMD.
 */
class LMDLocalization : public LocalizationBase {
  struct PredictorHandler {
    std::shared_ptr<Predictor> predictor;
    std::future<apollo::common::Status> fut;

    PredictorHandler() = default;

    explicit PredictorHandler(Predictor *predictor) {
      this->predictor.reset(predictor);
    }

    bool Busy() const {
      if (!fut.valid()) return false;
      auto s = fut.wait_for(std::chrono::seconds::zero());
      return s != std::future_status::ready;
    }
  };

  template <class Message>
  struct MessageReciever {
    std::list<Message> msg_list;
    const PredictorHandler *ph = nullptr;
    std::function<void(const Message &)> on_msg_recieved;

    void Set(const PredictorHandler &ph,
             std::function<void(const Message &)> &&on_msg_recieved) {
      this->ph = &ph;
      this->on_msg_recieved = std::move(on_msg_recieved);
    }

    void OnMessage(const Message &msg) {
      if (ph != nullptr) {
        if (!ph->Busy()) {
          for (const auto &msg : msg_list) {
            on_msg_recieved(msg);
          }
          msg_list.clear();
          on_msg_recieved(msg);
        } else {
          msg_list.emplace_back(msg);
        }
      }
    }
  };

 public:
  LMDLocalization();
  virtual ~LMDLocalization();

  /**
   * @brief Module start function.
   * @return Start status.
   */
  apollo::common::Status Start() override;

  /**
   * @brief Module stop function.
   * @return Stop status.
   */
  apollo::common::Status Stop() override;

 private:
  void OnImu(const CorrectedImu &imu);
  void OnRawImu(const apollo::drivers::gnss::Imu &imu);
  void OnGps(const Gps &gps);
  void OnChassis(const apollo::canbus::Chassis &chassis);
  void OnPerceptionObstacles(
      const apollo::perception::PerceptionObstacles &obstacles);
  void OnTimer(const ros::TimerEvent &event);

  void Predicting();
  void RunWatchDog();

 private:
  ros::Timer timer_;
  apollo::common::monitor::MonitorLogger monitor_logger_;
  const std::vector<double> map_offset_;
  std::map<std::string, PredictorHandler> predictors_;
  MessageReciever<CorrectedImu> imu_reciever_;
  MessageReciever<apollo::drivers::gnss::Imu> imu_reciever1_;
  MessageReciever<Gps> gps_reciever_;
  MessageReciever<apollo::canbus::Chassis> filtered_imu_reciever_;
  MessageReciever<apollo::perception::PerceptionObstacles> perception_reciever_;
};  // namespace localization

}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_LMD_LMD_LOCALIZATION_H_
