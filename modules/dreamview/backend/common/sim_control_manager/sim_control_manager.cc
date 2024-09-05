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
#include "modules/dreamview/backend/common/sim_control_manager/sim_control_manager.h"

#include "nlohmann/json.hpp"

namespace apollo {
namespace dreamview {

using apollo::canbus::Chassis;
using apollo::common::ErrorCode;
using apollo::control::ControlCommand;
using Json = nlohmann::json;

SimControlManager::SimControlManager() {}

std::string SimControlManager::Name() const {
  return FLAGS_sim_control_module_name;
}

Json SimControlManager::LoadDynamicModels() {
  auto *model_factory = DynamicModelFactory::Instance();
  Json result = model_factory->RegisterDynamicModels();
  return result;
}

void SimControlManager::Reset() {
  if (IsEnabled() && model_ptr_) {
    model_ptr_->Reset();
  }
}

void SimControlManager::ResetDynamicModel() {
  if (!current_dynamic_model_.empty()) {
    if (!model_ptr_) {
      auto *model_factory = DynamicModelFactory::Instance();
      model_ptr_ = model_factory->GetModelType(current_dynamic_model_);
    }
    if (model_ptr_) {
      model_ptr_->Stop();
    }
  }
  return;
}

bool SimControlManager::AddDynamicModel(const std::string &dynamic_model_name) {
  if (IsEnabled()) {
    auto *model_factory = DynamicModelFactory::Instance();
    return model_factory->RegisterDynamicModel(dynamic_model_name);
  } else {
    AERROR << "Sim control manager is not enabled! Can not download dynamic "
              "model to local!";
    return false;
  }
}

bool SimControlManager::ChangeDynamicModel(
    const std::string &dynamic_model_name) {
  auto *model_factory = DynamicModelFactory::Instance();
  auto next_model_ptr_ = model_factory->GetModelType(dynamic_model_name);
  if (!next_model_ptr_) {
    AERROR << "Can not get dynamic model to start.Use original dynamic model!";
    return false;
  }
  ResetDynamicModel();
  model_ptr_ = next_model_ptr_;
  next_model_ptr_ = nullptr;
  model_ptr_->Start();
  current_dynamic_model_ = dynamic_model_name;
  return true;
}

bool SimControlManager::DeleteDynamicModel(
    const std::string &dynamic_model_name) {
  auto *model_factory = DynamicModelFactory::Instance();
  return model_factory->UnregisterDynamicModel(dynamic_model_name);
}

void SimControlManager::Start() {
  // enabled_ only performs switching function
  if (!enabled_) {
    enabled_ = true;
  }
}

void SimControlManager::ReSetPoinstion(double x, double y, double heading) {
  if (!IsEnabled() || !model_ptr_) {
    AERROR << "Sim control is invalid,Failed to ReSetPoinstion!";
    return;
  }
  model_ptr_->ReSetPoinstion(x, y, heading);
  return;
}

void SimControlManager::Restart(double x, double y, double v, double a) {
  // reset start point for dynamic model.
  if (!IsEnabled() || !model_ptr_) {
    AERROR << "Sim control is invalid,Failed to restart!";
    return;
  }
  model_ptr_->Stop();
  model_ptr_->Start(x, y, v, a);
  return;
}

void SimControlManager::Restart() {
  // reset start point for dynamic model.
  if (!IsEnabled() || !model_ptr_) {
    AERROR << "Sim control is invalid,Failed to restart!";
    return;
  }
  model_ptr_->Stop();
  model_ptr_->Start();
  return;
}

void SimControlManager::RunOnce() { model_ptr_->RunOnce(); }

void SimControlManager::Stop() {
  if (enabled_) {
    enabled_ = false;
    ResetDynamicModel();
    std::system(FLAGS_sim_obstacle_stop_command.data());
    model_ptr_ = nullptr;
    current_dynamic_model_ = "";
  }
}

}  // namespace dreamview
}  // namespace apollo
