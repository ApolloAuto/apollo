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
#include "modules/dreamview/backend/sim_control_manager/sim_control_manager.h"

#include "nlohmann/json.hpp"

namespace apollo
{
  namespace dreamview
  {

    using apollo::canbus::Chassis;
    using apollo::common::ErrorCode;
    using apollo::control::ControlCommand;
    using Json = nlohmann::json;

    std::string SimControlManager::Name() const
    {
      return FLAGS_sim_control_module_name;
    }

    Json SimControlManager::LoadDynamicModels()
    {
      auto *model_factory = DynamicModelFactory::Instance();
      return model_factory->RegisterDynamicModels();
    }

    void SimControlManager::Reset(){
      if(IsEnabled() && model_ptr_){
        model_ptr_->Reset();
      }
    }

    void SimControlManager::ResetDynamicModel()
    {
      if (!current_dynamic_model_.empty())
      {
        if (!model_ptr_)
        {
          auto *model_factory = DynamicModelFactory::Instance();
          model_ptr_ = model_factory->GetModelType(current_dynamic_model_);
        }
        if (model_ptr_)
        {
          model_ptr_->Stop();
        }
      }
      return;
    }

    bool SimControlManager::AddDynamicModel(std::string &dynamic_model_name)
    {
      if (IsEnabled())
      {
        auto *model_factory = DynamicModelFactory::Instance();
        return model_factory->RegisterDynamicModel(dynamic_model_name);
      }
      else
      {
        AERROR << "Sim control manager is not enabled! Can not download dynamic model to local!";
        return false;
      }
    }

    bool SimControlManager::ChangeDynamicModel(std::string &dynamic_model_name)
    {
      ResetDynamicModel();
      auto *model_factory = DynamicModelFactory::Instance();
      model_ptr_ = model_factory->GetModelType(dynamic_model_name);
      if (!model_ptr_)
      {
        AERROR << "Can not get dynamic model to start.Reset it to original dynamic model!";
        model_ptr_ = model_factory->GetModelType(current_dynamic_model_);
        // 重新启动之前的动力学模型
        if (!model_ptr_)
        {
          model_ptr_->Start();
        }
        return false;
      }
      model_ptr_->Start();
      current_dynamic_model_ = dynamic_model_name;
      return true;
    }

    bool SimControlManager::DeleteDynamicModel(std::string &dynamic_model_name)
    {
      auto *model_factory = DynamicModelFactory::Instance();
      return model_factory->UnregisterDynamicModel(dynamic_model_name);
    }

    void SimControlManager::Start()
    {
      // enabled_仅承担开关作用
      if (!enabled_)
      {
        enabled_ = true;
      }
    }

    void SimControlManager::Restart(double x, double y)
    {
      // 只涉及模型，不涉及全局管理。只是重新开启更新起点
      if(!IsEnabled() || !model_ptr_){
        AERROR<<"Sim control is invalid,Failed to restart!";
      }
      model_ptr_->Stop();
      model_ptr_->Start(x,y);
      return;
    }

    void SimControlManager::RunOnce() { model_ptr_->RunOnce(); }

    void SimControlManager::Stop()
    {
      // todo: stop 需要好好补充！ 关闭太多相关的东西了
      if (enabled_)
      {
        //...do something
        enabled_ = false;
        // 获取当前 reset
        ResetDynamicModel();
        // kill sim obstacle
        std::system(FLAGS_sim_obstacle_stop_command.data());
        model_ptr_ = nullptr;
        current_dynamic_model_ = "";
      }
    }

  } // namespace dreamview
} // namespace apollo
