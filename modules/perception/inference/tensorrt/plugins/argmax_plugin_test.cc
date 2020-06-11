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

#include "modules/perception/inference/tensorrt/plugins/argmax_plugin.h"

#include "gtest/gtest.h"

TEST(ArgmaxPluginsTest, test) {
  {
    apollo::perception::inference::ArgMaxParameter argmax_param;
    argmax_param.set_out_max_val(true);
    nvinfer1::Dims in_dims;
    in_dims.nbDims = 3;
    in_dims.d[0] = 8;
    in_dims.d[1] = 10;
    in_dims.d[2] = 20;
    apollo::perception::inference::ArgMax1Plugin arg_plugin(argmax_param,
                                                            in_dims);
    auto out_dims = arg_plugin.getOutputDimensions(0, &in_dims, 3);
    EXPECT_EQ(out_dims.d[0], 2);
  }
  {
    apollo::perception::inference::ArgMaxParameter argmax_param;
    argmax_param.set_out_max_val(false);
    nvinfer1::Dims in_dims;
    in_dims.nbDims = 3;
    in_dims.d[0] = 8;
    in_dims.d[1] = 10;
    in_dims.d[2] = 20;
    apollo::perception::inference::ArgMax1Plugin arg_plugin(argmax_param,
                                                            in_dims);
    auto out_dims = arg_plugin.getOutputDimensions(0, &in_dims, 3);
    EXPECT_EQ(out_dims.d[0], 1);
  }
}
