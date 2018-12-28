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

#include "modules/localization/lmd/lmd_localization.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/util/thread_pool.h"
#include "modules/localization/common/localization_gflags.h"
#include "modules/localization/lmd/predictor/filter/predictor_filtered_imu.h"
#include "modules/localization/lmd/predictor/output/predictor_output.h"
#include "modules/localization/lmd/predictor/output/predictor_print_error.h"
#include "modules/localization/lmd/predictor/perception/predictor_perception.h"
#include "modules/localization/lmd/predictor/raw/predictor_gps.h"
#include "modules/localization/lmd/predictor/raw/predictor_imu.h"

namespace apollo {
namespace localization {

using apollo::canbus::Chassis;
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::adapter::AdapterManager;
using apollo::common::adapter::ChassisAdapter;
using apollo::common::adapter::GpsAdapter;
using apollo::common::adapter::ImuAdapter;
using apollo::common::monitor::MonitorMessageItem;
using apollo::common::util::ThreadPool;
using apollo::drivers::gnss::Imu;
using apollo::perception::PerceptionObstacles;

namespace {
constexpr double kDefaultMemoryCycle = 1.0;
constexpr int kDefaultThreadPoolSize = 2;
}  // namespace

LMDLocalization::LMDLocalization()
    : monitor_logger_(MonitorMessageItem::LOCALIZATION),
      map_offset_{FLAGS_map_offset_x, FLAGS_map_offset_y, FLAGS_map_offset_z} {}

LMDLocalization::~LMDLocalization() {}

Status LMDLocalization::Start() {
  auto publish_func = [&](double timestamp_sec, const Pose &pose) {
    // prepare localization message
    LocalizationEstimate localization;
    AdapterManager::FillLocalizationHeader(FLAGS_localization_module_name,
                                           &localization);
    localization.set_measurement_time(timestamp_sec);
    localization.mutable_pose()->CopyFrom(pose);
    localization.mutable_pose()->mutable_position()->set_x(pose.position().x() -
                                                           map_offset_[0]);
    localization.mutable_pose()->mutable_position()->set_y(pose.position().y() -
                                                           map_offset_[1]);
    localization.mutable_pose()->mutable_position()->set_z(pose.position().z() -
                                                           map_offset_[2]);

    // publish localization messages
    AdapterManager::PublishLocalization(localization);
    PublishPoseBroadcastTF(localization);
    return Status::OK();
  };

  // initialize predictors
  auto add_predictor = [&](Predictor *predictor) -> PredictorHandler * {
    return &predictors_.emplace(predictor->Name(), PredictorHandler(predictor))
                .first->second;
  };

  // gps
  auto *predictor_gps = new PredictorGps(kDefaultMemoryCycle);
  gps_reciever_.Set(*add_predictor(predictor_gps),
                    [=](const Gps &msg) { predictor_gps->UpdateGps(msg); });

  // imu
  auto *predictor_imu = new PredictorImu(kDefaultMemoryCycle);
  auto &ph_imu = *add_predictor(predictor_imu);
  imu_reciever_.Set(
      ph_imu, [=](const CorrectedImu &msg) { predictor_imu->UpdateImu(msg); });
  imu_reciever1_.Set(ph_imu,
                     [=](const Imu &msg) { predictor_imu->UpdateRawImu(msg); });

  // filtered_imu
  auto *predictor_filtered_imu = new PredictorFilteredImu(kDefaultMemoryCycle);
  filtered_imu_reciever_.Set(
      *add_predictor(predictor_filtered_imu),
      [=](const Chassis &msg) { predictor_filtered_imu->UpdateChassis(msg); });

  // perception
  auto *predictor_perception = new PredictorPerception(kDefaultMemoryCycle);
  perception_reciever_.Set(
      *add_predictor(predictor_perception),
      [=](const PerceptionObstacles &msg) {
        if (!msg.has_header() || !msg.header().has_timestamp_sec() ||
            !msg.has_lane_marker()) {
          return;
        }
        predictor_perception->UpdateLaneMarkers(msg.header().timestamp_sec(),
                                                msg.lane_marker());
      });

  // output
  auto *predictor_output =
      new PredictorOutput(kDefaultMemoryCycle, publish_func);
  add_predictor(predictor_output);

  // print_error
  auto *predictor_print_error = new PredictorPrintError(kDefaultMemoryCycle);
  add_predictor(predictor_print_error);

  // check predictors' dep
  for (const auto &p : predictors_) {
    auto &ph = p.second;
    for (const auto &dep_p : ph.predictor->DepPredicteds()) {
      if (predictors_.find(dep_p.first) == predictors_.end()) {
        AERROR << "Can not find predictor[" << dep_p.first << "]";
        predictors_.clear();
        return Status(
            ErrorCode::LOCALIZATION_ERROR,
            "Can not find predictor with name [\"" + dep_p.first + "\"]");
      }
    }
  }

  // initialize thread pool
  ThreadPool::Init(kDefaultThreadPoolSize);
  // initialize adapter manager
  AdapterManager::Init(FLAGS_lmd_adapter_config_file);
  // Imu
  if (!AdapterManager::GetImu()) {
    AERROR << "IMU input not initialized. Check your adapter.conf file!";
    return Status(common::LOCALIZATION_ERROR_MSG, "no IMU adapter");
  }
  AdapterManager::AddImuCallback(&LMDLocalization::OnImu, this);
  // Raw Imu
  if (!AdapterManager::GetRawImu()) {
    AERROR << "Raw IMU input not initialized. Check your adapter.conf file!";
    return Status(common::LOCALIZATION_ERROR_MSG, "no Raw IMU adapter");
  }
  AdapterManager::AddRawImuCallback(&LMDLocalization::OnRawImu, this);
  // Gps
  if (!AdapterManager::GetGps()) {
    AERROR << "Gps input not initialized. Check your adapter.conf file!";

    return Status(common::LOCALIZATION_ERROR_MSG, "no Gps adapter");
  }
  AdapterManager::AddGpsCallback(&LMDLocalization::OnGps, this);
  // Chassis
  if (!AdapterManager::GetChassis()) {
    AERROR << "Chassis input not initialized. Check your adapter.conf file!";
    return Status(common::LOCALIZATION_ERROR_MSG, "no Chassis adapter");
  }
  AdapterManager::AddChassisCallback(&LMDLocalization::OnChassis, this);
  // Perception
  if (!AdapterManager::GetPerceptionObstacles()) {
    AERROR
        << "PerceptionObstacles input not initialized. Check your adapter.conf "
           "file!";
    return Status(common::LOCALIZATION_ERROR_MSG,
                  "no PerceptionObstacles adapter");
  }
  AdapterManager::AddPerceptionObstaclesCallback(
      &LMDLocalization::OnPerceptionObstacles, this);
  // start ROS timer, one-shot = false, auto-start = true
  const double duration = 1.0 / FLAGS_localization_publish_freq;
  timer_ = AdapterManager::CreateTimer(ros::Duration(duration),
                                       &LMDLocalization::OnTimer, this);

  tf2_broadcaster_.reset(new tf2_ros::TransformBroadcaster);

  return Status::OK();
}

Status LMDLocalization::Stop() {
  timer_.stop();
  ThreadPool::Stop();
  return Status::OK();
}

void LMDLocalization::OnImu(const CorrectedImu &imu) {
  imu_reciever_.OnMessage(imu);
  // predicting
  Predicting();
}

void LMDLocalization::OnRawImu(const Imu &imu) {
  imu_reciever1_.OnMessage(imu);
  // predicting
  Predicting();
}

void LMDLocalization::OnGps(const Gps &gps) {
  gps_reciever_.OnMessage(gps);
  // predicting
  Predicting();
}

void LMDLocalization::OnChassis(const Chassis &chassis) {
  filtered_imu_reciever_.OnMessage(chassis);
  // predicting
  Predicting();
}

void LMDLocalization::OnPerceptionObstacles(
    const PerceptionObstacles &obstacles) {
  perception_reciever_.OnMessage(obstacles);
  // predicting
  Predicting();
}

void LMDLocalization::OnTimer(const ros::TimerEvent &event) {
  // predicting
  Predicting();
  // watch dog
  RunWatchDog();
}

void LMDLocalization::Predicting() {
  bool finish = false;
  while (!finish) {
    finish = true;

    // update predicteds from deps
    for (auto &p : predictors_) {
      auto &ph = p.second;
      if (!ph.Busy()) {
        for (const auto &dep_p : ph.predictor->DepPredicteds()) {
          const auto &dep_name = dep_p.first;
          const auto &dep_ph = predictors_[dep_name];
          if (dep_ph.Busy()) {
            continue;
          }
          const auto &dep_predicted = dep_p.second;
          if (dep_predicted.empty() ||
              dep_predicted.Older(dep_ph.predictor->Predicted())) {
            ph.predictor->UpdateDepPredicted(dep_name,
                                             dep_ph.predictor->Predicted());
          }
        }
      }
    }

    // predict
    for (auto &p : predictors_) {
      auto &ph = p.second;
      if (!ph.Busy() && ph.predictor->Updateable()) {
        finish = false;

        if (ph.predictor->UpdatingOnAdapterThread()) {
          ph.predictor->Update();
        } else {
          auto predictor = ph.predictor;
          ph.fut =
              ThreadPool::pool()->push([=](int thread_id) mutable -> Status {
                return predictor->Update();
              });
        }
      }
    }
  }
}

void LMDLocalization::RunWatchDog() {
  if (!FLAGS_enable_watchdog) return;

  // Add code to implement watch dog
}

}  // namespace localization
}  // namespace apollo
