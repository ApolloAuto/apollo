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

#include "modules/perception/inference/tensorrt/plugins/slice_plugin.h"

#include "gtest/gtest.h"

#include "modules/perception/proto/rt.pb.h"

TEST(SlicePluginsTest, test) {
  apollo::perception::inference::SliceParameter slice_param;
  slice_param.add_slice_point(3);
  nvinfer1::Dims in_dims;
  in_dims.nbDims = 5;
  in_dims.d[0] = 8;
  in_dims.d[1] = 10;
  in_dims.d[2] = 20;
  in_dims.d[3] = 30;
  in_dims.d[4] = 40;

  apollo::perception::inference::SLICEPlugin slice_plugin(slice_param, in_dims);
  auto out_dims = slice_plugin.getOutputDimensions(0, &in_dims, 5);
  EXPECT_EQ(out_dims.d[0], 3);
  EXPECT_EQ(out_dims.d[1], 10);
  EXPECT_EQ(out_dims.d[2], 20);
  EXPECT_EQ(out_dims.d[3], 30);
  EXPECT_EQ(out_dims.d[4], 40);
  EXPECT_EQ(slice_plugin.getNbOutputs(), 2);

  out_dims = slice_plugin.getOutputDimensions(1, &in_dims, 5);
  EXPECT_EQ(out_dims.d[0], 5);
  EXPECT_EQ(out_dims.d[1], 10);
  EXPECT_EQ(out_dims.d[2], 20);
  EXPECT_EQ(out_dims.d[3], 30);
  EXPECT_EQ(out_dims.d[4], 40);

  slice_param.set_axis(2);
  slice_param.add_slice_point(5);
  apollo::perception::inference::SLICEPlugin slice_plugin2(slice_param,
                                                           in_dims);
  out_dims = slice_plugin2.getOutputDimensions(0, &in_dims, 5);
  EXPECT_EQ(out_dims.d[0], 8);
  EXPECT_EQ(out_dims.d[1], 3);
  EXPECT_EQ(out_dims.d[2], 20);
  EXPECT_EQ(out_dims.d[3], 30);
  EXPECT_EQ(out_dims.d[4], 40);

  out_dims = slice_plugin2.getOutputDimensions(1, &in_dims, 5);
  EXPECT_EQ(out_dims.d[0], 8);
  EXPECT_EQ(out_dims.d[1], 2);
  EXPECT_EQ(out_dims.d[2], 20);
  EXPECT_EQ(out_dims.d[3], 30);
  EXPECT_EQ(out_dims.d[4], 40);

  out_dims = slice_plugin2.getOutputDimensions(2, &in_dims, 5);
  EXPECT_EQ(out_dims.d[0], 8);
  EXPECT_EQ(out_dims.d[1], 5);
  EXPECT_EQ(out_dims.d[2], 20);
  EXPECT_EQ(out_dims.d[3], 30);
  EXPECT_EQ(out_dims.d[4], 40);
}
