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
#pragma once

#include <memory>
#include "cyber/common/macros.h"
#include "nlohmann/json.hpp"

#include "modules/dreamview/backend/sim_control_manager/core/sim_control_base.h"
#include "modules/dreamview/backend/sim_control_manager/core/sim_control_with_model_base.h"

namespace apollo
{
  namespace dreamview
  {

    class DynamicModelFactory
    {
    public:
      // 改动1：add dynamic model name param
      bool GetModelType(std::string dynamic_model_name, SimControlBase* dynamic_model);
          nlohmann::json RegisterDynamicModels();
      bool RegisterDynamicModel(std::string &dm_library_path, std::string &dm_name);
      void UnregisterDynamicModel(std::string &dynamic_model_name);
      void GetDynamicModelPath(std::string &dynamic_model_name, std::string &path, bool get_library = true);

    private:
      std::string dynamic_model_local_path_;
      DECLARE_SINGLETON(DynamicModelFactory);
    };

  } // namespace dreamview
} // namespace apollo
