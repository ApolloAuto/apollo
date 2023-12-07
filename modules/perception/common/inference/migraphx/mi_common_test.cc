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

#include "modules/perception/common/inference/migraphx/mi_common.h"

#include "gtest/gtest.h"

namespace apollo {
namespace perception {
namespace inference {

TEST(RTModifyPoolingParamTest, test) {
  {
    PoolingParameter pool_param;
    pool_param.set_kernel_size(1);
    pool_param.set_stride(1);
    ACHECK(modify_pool_param(&pool_param));
    EXPECT_EQ(pool_param.kernel_w(), 1);
    EXPECT_EQ(pool_param.kernel_h(), 1);
  }
  {
    PoolingParameter pool_param;
    pool_param.set_kernel_h(0);
    ACHECK(!modify_pool_param(&pool_param));
  }
  {
    PoolingParameter pool_param;
    pool_param.set_kernel_w(0);
    ACHECK(!modify_pool_param(&pool_param));
  }
  {
    PoolingParameter pool_param;
    pool_param.set_stride(1);
    pool_param.set_kernel_size(1);
    ACHECK(modify_pool_param(&pool_param));
    EXPECT_EQ(pool_param.stride_w(), 1);
    EXPECT_EQ(pool_param.stride_h(), 1);
  }
  {
    PoolingParameter pool_param;
    pool_param.set_kernel_size(1);
    pool_param.set_stride_h(0);
    ACHECK(!modify_pool_param(&pool_param));
  }
  {
    PoolingParameter pool_param;
    pool_param.set_stride_w(0);
    pool_param.set_kernel_size(1);
    ACHECK(!modify_pool_param(&pool_param));
  }
  {
    PoolingParameter pool_param;
    pool_param.set_pad(1);
    pool_param.set_kernel_size(1);
    pool_param.set_stride(1);
    ACHECK(modify_pool_param(&pool_param));
    EXPECT_EQ(pool_param.pad_h(), 1);
    EXPECT_EQ(pool_param.pad_w(), 1);
  }
  {
    PoolingParameter pool_param;
    pool_param.set_kernel_size(1);
    pool_param.set_stride(1);
    ACHECK(modify_pool_param(&pool_param));
    EXPECT_EQ(pool_param.pad_h(), 0);
    EXPECT_EQ(pool_param.pad_w(), 0);
  }
}

TEST(RTParseConvParamTest, test) {
  {
    ConvolutionParameter conv_param;
    conv_param.add_kernel_size(3);
    conv_param.add_pad(1);
    conv_param.add_stride(1);
    apollo::perception::inference::ConvParam tensorrt_param;
    ACHECK(ParserConvParam(conv_param, &tensorrt_param));
    EXPECT_EQ(tensorrt_param.kernel_h, 3);
    EXPECT_EQ(tensorrt_param.kernel_w, 3);
    EXPECT_EQ(tensorrt_param.padding_h, 1);
    EXPECT_EQ(tensorrt_param.padding_w, 1);
    EXPECT_EQ(tensorrt_param.stride_h, 1);
    EXPECT_EQ(tensorrt_param.stride_w, 1);
  }
  {
    ConvolutionParameter conv_param;
    conv_param.clear_kernel_size();
    apollo::perception::inference::ConvParam tensorrt_param;
    ACHECK(!ParserConvParam(conv_param, &tensorrt_param));
  }
  {
    ConvolutionParameter conv_param;
    conv_param.clear_kernel_size();
    conv_param.set_kernel_h(3);
    apollo::perception::inference::ConvParam tensorrt_param;
    ACHECK(!ParserConvParam(conv_param, &tensorrt_param));
  }
  {
    ConvolutionParameter conv_param;
    conv_param.clear_kernel_size();
    conv_param.set_kernel_w(3);
    apollo::perception::inference::ConvParam tensorrt_param;
    ACHECK(!ParserConvParam(conv_param, &tensorrt_param));
  }
  {
    ConvolutionParameter conv_param;
    conv_param.add_kernel_size(3);
    conv_param.set_pad_h(1);
    conv_param.add_stride(1);
    conv_param.add_dilation(0);
    apollo::perception::inference::ConvParam tensorrt_param;
    ACHECK(ParserConvParam(conv_param, &tensorrt_param));
    EXPECT_EQ(tensorrt_param.kernel_h, 3);
    EXPECT_EQ(tensorrt_param.kernel_w, 3);
    EXPECT_EQ(tensorrt_param.padding_h, 1);
    EXPECT_EQ(tensorrt_param.padding_w, 0);
    EXPECT_EQ(tensorrt_param.stride_h, 1);
    EXPECT_EQ(tensorrt_param.stride_w, 1);
  }
  {
    ConvolutionParameter conv_param;
    conv_param.add_kernel_size(3);
    conv_param.set_pad_w(1);
    conv_param.add_stride(1);
    conv_param.add_dilation(0);
    apollo::perception::inference::ConvParam tensorrt_param;
    ACHECK(ParserConvParam(conv_param, &tensorrt_param));
    EXPECT_EQ(tensorrt_param.kernel_h, 3);
    EXPECT_EQ(tensorrt_param.kernel_w, 3);
    EXPECT_EQ(tensorrt_param.padding_h, 0);
    EXPECT_EQ(tensorrt_param.padding_w, 1);
    EXPECT_EQ(tensorrt_param.stride_h, 1);
    EXPECT_EQ(tensorrt_param.stride_w, 1);
  }
  {
    ConvolutionParameter conv_param;
    conv_param.add_kernel_size(3);
    conv_param.add_pad(1);
    conv_param.set_pad_w(1);
    conv_param.add_dilation(0);
    apollo::perception::inference::ConvParam tensorrt_param;
    ACHECK(!ParserConvParam(conv_param, &tensorrt_param));
  }
  {
    ConvolutionParameter conv_param;
    conv_param.add_kernel_size(3);
    conv_param.add_pad(1);
    conv_param.set_pad_h(1);
    conv_param.add_dilation(0);
    apollo::perception::inference::ConvParam tensorrt_param;
    ACHECK(!ParserConvParam(conv_param, &tensorrt_param));
  }
  {
    ConvolutionParameter conv_param;
    conv_param.add_kernel_size(3);
    conv_param.add_pad(1);
    conv_param.add_stride(1);
    conv_param.set_stride_h(1);
    conv_param.add_dilation(0);
    apollo::perception::inference::ConvParam tensorrt_param;
    ACHECK(!ParserConvParam(conv_param, &tensorrt_param));
  }
  {
    ConvolutionParameter conv_param;
    conv_param.add_kernel_size(3);
    conv_param.add_pad(1);
    conv_param.add_stride(1);
    conv_param.set_stride_w(1);
    conv_param.add_dilation(0);
    apollo::perception::inference::ConvParam tensorrt_param;
    ACHECK(!ParserConvParam(conv_param, &tensorrt_param));
  }
  {
    ConvolutionParameter conv_param;
    conv_param.add_kernel_size(3);
    conv_param.add_pad(1);
    conv_param.set_stride_h(1);
    conv_param.add_dilation(0);
    apollo::perception::inference::ConvParam tensorrt_param;
    ACHECK(!ParserConvParam(conv_param, &tensorrt_param));
  }
  {
    ConvolutionParameter conv_param;
    conv_param.add_kernel_size(3);
    conv_param.add_pad(1);
    conv_param.set_stride_w(1);
    conv_param.add_dilation(0);
    apollo::perception::inference::ConvParam tensorrt_param;
    ACHECK(!ParserConvParam(conv_param, &tensorrt_param));
  }
}

}  // namespace inference
}  // namespace perception
}  // namespace apollo
