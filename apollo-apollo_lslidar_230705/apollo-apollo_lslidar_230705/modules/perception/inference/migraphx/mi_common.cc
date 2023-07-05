/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/inference/migraphx/mi_common.h"

#include <utility>

#include "absl/strings/str_cat.h"

namespace apollo {
namespace perception {
namespace inference {

void ParseNetParam(const NetParameter &net_param,
                   TensorDimsMap *tensor_dims_map,
                   std::map<std::string, std::string> *tensor_modify_map,
                   std::vector<LayerParameter> *order) {
  for (int i = 0; i < net_param.layer_size(); ++i) {
    LayerParameter tensorrt_layer_param = net_param.layer(i);
    if (tensorrt_layer_param.type() == "Input") {
      InputParameter input = tensorrt_layer_param.input_param();
      for (int j = 0; j < tensorrt_layer_param.top().size(); ++j) {
        TensorDims dims{static_cast<int>(input.shape(j).dim(0)),
                        static_cast<int>(input.shape(j).dim(1)),
                        static_cast<int>(input.shape(j).dim(2)),
                        static_cast<int>(input.shape(j).dim(3))};
        auto name = tensorrt_layer_param.top(j);
        tensor_dims_map->insert(std::make_pair(name, dims));
        tensor_modify_map->insert(
            std::make_pair(name, tensorrt_layer_param.top(j)));
      }
    } else {
      order->push_back(tensorrt_layer_param);
    }
  }
}

bool ParserConvParam(const ConvolutionParameter &conv, ConvParam *param) {
  if (conv.has_kernel_h() || conv.has_kernel_w()) {
    if (conv.kernel_size_size() != 0) {
      return false;
    }
    param->kernel_h = conv.kernel_h();
    param->kernel_w = conv.kernel_w();
  } else {
    if (conv.kernel_size_size() < 1) {
      return false;
    }
    param->kernel_h = conv.kernel_size(0);
    param->kernel_w = (conv.kernel_size_size() > 1 ? conv.kernel_size(1)
                                                   : conv.kernel_size(0));
  }
  if (param->kernel_h == 0 || param->kernel_w == 0) {
    return false;
  }

  if (conv.has_pad_h() || conv.has_pad_w()) {
    if (conv.pad_size() != 0) {
      return false;
    }
    param->padding_h = conv.pad_h();
    param->padding_w = conv.pad_w();
  } else {
    param->padding_h = (conv.pad().empty() ? 0 : conv.pad(0));
    param->padding_w = (conv.pad_size() > 1 ? conv.pad(1) : param->padding_h);
  }

  if (conv.has_stride_h() || conv.has_stride_w()) {
    if (conv.stride_size() != 0) {
      return false;
    }
    param->stride_h = conv.stride_h();
    param->stride_w = conv.stride_w();
  } else {
    param->stride_h = (conv.stride().empty() ? 1 : conv.stride(0));
    param->stride_w =
        (conv.stride_size() > 1 ? conv.stride(1) : param->stride_h);
  }
  if (param->stride_h == 0 || param->stride_w == 0) {
    return false;
  }

  param->dilation = conv.dilation().empty() ? 1 : conv.dilation(0);
  param->group = conv.has_group() ? conv.group() : 1;
  return true;
}

bool modify_pool_param(PoolingParameter *pool_param) {
  if (pool_param->has_kernel_size()) {
    pool_param->set_kernel_h(pool_param->kernel_size());
    pool_param->set_kernel_w(pool_param->kernel_size());
  }
  if (pool_param->kernel_w() == 0 || pool_param->kernel_h() == 0) {
    return false;
  }
  if (pool_param->has_pad()) {
    pool_param->set_pad_h(pool_param->pad());
    pool_param->set_pad_w(pool_param->pad());
  }
  if (pool_param->has_stride()) {
    pool_param->set_stride_h(pool_param->stride());
    pool_param->set_stride_w(pool_param->stride());
  }
  if (pool_param->stride_w() == 0 || pool_param->stride_h() == 0) {
    return false;
  }
  return true;
}

}  // namespace inference
}  // namespace perception
}  // namespace apollo
