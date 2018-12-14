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
#include "modules/localization/lmd/predictor/output/predictor_output.h"
#include "modules/localization/lmd/predictor/output/predictor_print_error.h"
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
using apollo::perception::PerceptionObstacles;

namespace {
constexpr double kDefaultMemoryCycle = 2.0;
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
  Predictor *predictor = new PredictorGps(kDefaultMemoryCycle);
  predictors_.emplace(predictor->Name(), PredictorHandler(predictor));
  gps_ = &predictors_[predictor->Name()];
  predictor = new PredictorImu(kDefaultMemoryCycle);
  predictors_.emplace(predictor->Name(), PredictorHandler(predictor));
  imu_ = &predictors_[predictor->Name()];
  predictor = new PredictorOutput(kDefaultMemoryCycle, publish_func);
  predictors_.emplace(predictor->Name(), PredictorHandler(predictor));
  output_ = &predictors_[predictor->Name()];
  predictor = new PredictorPrintError(kDefaultMemoryCycle);
  predictors_.emplace(predictor->Name(), PredictorHandler(predictor));

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
  AdapterManager::AddImuCallback(&LMDLocalization::OnImu, this);
  AdapterManager::AddGpsCallback(&LMDLocalization::OnGps, this);
  AdapterManager::AddChassisCallback(&LMDLocalization::OnChassis, this);
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
  if (!imu_->Busy()) {
    // update messages
    auto *predictor = static_cast<PredictorImu *>(imu_->predictor.get());
    for (const auto &imu : imu_list_) {
      predictor->UpdateImu(imu);
    }
    imu_list_.clear();
    predictor->UpdateImu(imu);

    // predicting
    Predicting();
  } else {
    imu_list_.emplace_back(imu);
  }
}

void LMDLocalization::OnGps(const Gps &gps) {
  if (!gps_->Busy()) {
    // update messages
    auto *predictor = static_cast<PredictorGps *>(gps_->predictor.get());
    for (const auto &gps : gps_list_) {
      predictor->UpdateGps(gps);
    }
    gps_list_.clear();
    predictor->UpdateGps(gps);

    // predicting
    Predicting();
  } else {
    gps_list_.emplace_back(gps);
  }
}

void LMDLocalization::OnChassis(const Chassis &chassis) {}

void LMDLocalization::OnPerceptionObstacles(
    const PerceptionObstacles &obstacles) {}

void LMDLocalization::OnTimer(const ros::TimerEvent &event) {
  // take a snapshot of the current received messages
  AdapterManager::Observe();

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
