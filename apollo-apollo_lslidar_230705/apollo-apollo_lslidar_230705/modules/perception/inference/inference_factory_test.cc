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

#include "modules/perception/inference/inference_factory.h"

#include "gtest/gtest.h"

#if GPU_PLATFORM == NVIDIA
  #include "modules/perception/inference/tensorrt/rt_net.h"
#elif GPU_PLATFORM == AMD
  #include "modules/perception/inference/migraphx/mi_net.h"
#endif

namespace apollo {
namespace perception {
namespace inference {

TEST(Inference_Factory, default) {}

}  // namespace inference
}  // namespace perception
}  // namespace apollo
