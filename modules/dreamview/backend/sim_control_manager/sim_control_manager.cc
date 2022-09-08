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
      // 获取json
      return model_factory->RegisterDynamicModels();
    }

    bool SimControlManager::ResetDynamicModel()
    {
      if (!current_dynamic_model_.empty())
      {
        if (!model_ptr_)
        {
          // 正常不应该出现这种情况
          auto *model_factory = DynamicModelFactory::Instance();
         if(!model_factory->GetModelType(current_dynamic_model_, model_ptr_)){
            current_dynamic_model_ = "";
            AERROR << "Can not get dynamic model to reset.";
            return false;
          }
        }
        // 旧dynamic model stop 注意-stop记得重置一些参数，保证下一次start又是对的
        model_ptr_->Stop();
      }
      return true;
    }

    bool SimControlManager::ChangeDynamicModel(std::string &dynamic_model_name)
    {
      // 感觉我这个设计模式谜之设计模式
      if (!ResetDynamicModel())
      {
        return false;
      }
      auto *model_factory = DynamicModelFactory::Instance();
      if (!dynamic_model_name.empty())
      {
        if (!model_factory->GetModelType(dynamic_model_name, model_ptr_))
        {
          // 改写dm的Init+start 变为新的start
          current_dynamic_model_ = "";
          AERROR << "Can not get dynamic model to start.";
          return false;
        }
        model_ptr_->Start();
      }
      else
      {
        // dm为空，关闭动力学模型仿真
        // 校验sim obstacle是否正在运行
        // 删除sim obstacle
        std::system(FLAGS_sim_obstacle_stop_command.data());
        model_ptr_ = nullptr;
      }
      current_dynamic_model_ = dynamic_model_name;
      return true;
    }

    void SimControlManager::DeleteDynamicModel(std::string &dynamic_model_name)
    {
      auto *model_factory = DynamicModelFactory::Instance();
      model_factory->UnregisterDynamicModel(dynamic_model_name);
      return;
    }

    // bool SimControlManager::Init(bool set_start_point, double start_velocity,
    //                              double start_acceleration, double start_heading) {
    //   auto* model_factory = DynamicModelFactory::Instance();
    //   model_ptr_ = model_factory->GetModelType();
    //   if (!model_ptr_) {
    //     AERROR << "Failed to create dynamic model.";
    //     // TODO(QiL): fulfill status error definition in apollo
    //     return false;
    //   }
    //   AINFO << "Dynamic Model is successfully created.";
    //   model_ptr_->Init(set_start_point, start_velocity, start_acceleration,
    //                    start_heading);
    //   return true;
    // }

    void SimControlManager::Start()
    {
      // enabled_仅承担开关作用
      if (!enabled_)
      {
        enabled_ = true;
      }
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
