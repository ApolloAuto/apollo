/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include <map>
#include <string>

#include <boost/function.hpp>

#include "modules/v2x/proto/fusion_params.pb.h"

#include "cyber/cyber.h"
#include "modules/v2x/fusion/configs/fusion_tracker_gflags.h"

#define PLUGIN_PARAMS(classname, conf_file, prototype)                       \
  class classname {                                                          \
   public:                                                                   \
    classname() {                                                            \
      file_path_ = FLAGS_config_path + "/" + conf_file;                      \
      cyber::common::GetProtoFromFile(file_path_, &params);                  \
    }                                                                        \
    ~classname() { cyber::common::SetProtoToASCIIFile(params, file_path_); } \
    prototype params;                                                        \
                                                                             \
   private:                                                                  \
    std::string file_path_;                                                  \
  };

namespace apollo {
namespace v2x {
namespace ft {

PLUGIN_PARAMS(FusionParams, FLAGS_fusion_conf_file, fusion::FusionParams)

class FTConfigManager {
 public:
  DECLARE_SINGLETON(FTConfigManager)
 public:
  FusionParams fusion_params_;
};

}  // namespace ft
}  // namespace v2x
}  // namespace apollo
