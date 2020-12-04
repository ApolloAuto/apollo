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

#include <map>
#include <string>
#include <vector>

#include "NvCaffeParser.h"
#include "NvInfer.h"
#include <cudnn.h>

#include "modules/perception/proto/rt.pb.h"

#include "cyber/common/log.h"
#include "modules/perception/base/common.h"

namespace apollo {
namespace perception {
namespace inference {

typedef std::map<std::string, std::vector<nvinfer1::Weights>> WeightMap;
typedef std::map<std::string, nvinfer1::ITensor *> TensorMap;
typedef std::map<std::string, nvinfer1::DimsCHW> TensorDimsMap;

nvinfer1::DimsCHW ReshapeDims(const nvinfer1::DimsCHW &dims,
                              const nvinfer1::DimsCHW &inputDims);
void ParseNetParam(const NetParameter &net_param,
                   TensorDimsMap *tensor_dims_map,
                   std::map<std::string, std::string> *tensor_modify_map,
                   std::vector<LayerParameter> *order);

bool modify_pool_param(PoolingParameter *pool_param);

struct ConvParam {
  int kernel_h;
  int kernel_w;
  int padding_h;
  int padding_w;
  int stride_h;
  int stride_w;
  int group;
  int dilation;
};

bool ParserConvParam(const ConvolutionParameter &conv, ConvParam *param);

}  // namespace inference
}  // namespace perception
}  // namespace apollo
