/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#ifndef MODULES_PERCEPTION_COMMON_CUDA_CONF_H_
#define MODULES_PERCEPTION_COMMON_CUDA_CONF_H_

#include "modules/common/macro.h"

namespace apollo {
namespace perception {

class CudaConfig {
public:
    CudaConfig();
public:
    static CudaConfig& instance();
    void initialize(unsigned int width, unsigned height);
    unsigned int width() const {return block_width_;}
    unsigned int height() const { return block_width_;}
private:
    unsigned int block_width_;
    unsigned int block_height_;
private:
    DISALLOW_COPY_AND_ASSIGN(CudaConfig)
};
};
};
#endif