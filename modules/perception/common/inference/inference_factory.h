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

#include <NvInferVersion.h>

#ifdef NV_TENSORRT_MAJOR
#if NV_TENSORRT_MAJOR == 8
#include "modules/perception/common/inference/tensorrt/rt_legacy.h"
#endif
#endif

#include <string>
#include <vector>

#include "modules/perception/common/proto/model_info.pb.h"

#include "modules/perception/common/inference/inference.h"

namespace apollo {
namespace perception {
namespace inference {

Inference *CreateInferenceByName(const std::string &frame_work,
                                 const std::string &proto_file,
                                 const std::string &weight_file,
                                 const std::vector<std::string> &outputs,
                                 const std::vector<std::string> &inputs,
                                 const std::string &model_root = "");

Inference *CreateInferenceByName(const common::Framework &frame_work,
                                 const std::string &proto_file,
                                 const std::string &weight_file,
                                 const std::vector<std::string> &outputs,
                                 const std::vector<std::string> &inputs,
                                 const std::string &model_root = "");

}  // namespace inference
}  // namespace perception
}  // namespace apollo
