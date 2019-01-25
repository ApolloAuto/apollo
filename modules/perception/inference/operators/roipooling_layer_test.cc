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

// This function was not verified yet

#include "modules/perception/inference/operators/roipooling_layer.h"

#include "gtest/gtest.h"

TEST(ROIPoolFloorTest, test) {
  int pooled_h = 7;
  int pooled_w = 7;
  bool use_floor = true;
  int feat_channel = 1;
  float spatial_scale = 16.0f;
  auto roi_layer = new apollo::perception::inference::ROIPoolingLayer<float>(
      pooled_h, pooled_w, use_floor, spatial_scale, feat_channel);
  std::vector<int> feat_shape{2, feat_channel, 30, 30};
  std::vector<int> rois_shape{4, 5};
  std::vector<int> top_shape{4, feat_channel, pooled_h, pooled_w};
  std::shared_ptr<apollo::perception::base::Blob<float>> feat_b;
  feat_b.reset(new apollo::perception::base::Blob<float>(feat_shape));
  std::shared_ptr<apollo::perception::base::Blob<float>> rois_b;
  rois_b.reset(new apollo::perception::base::Blob<float>(rois_shape));
  std::shared_ptr<apollo::perception::base::Blob<float>> top;
  top.reset(new apollo::perception::base::Blob<float>(top_shape));
  for (int b = 0; b < 2; b++) {
    for (int c = 0; c < feat_channel; c++) {
      int offset = feat_b->offset(b, c);
      for (int idx = 0; idx < feat_b->shape(2) * feat_b->shape(3); idx++) {
        feat_b->mutable_cpu_data()[offset + idx] = static_cast<float>(idx);
      }
    }
  }
  auto rois_data = rois_b->mutable_cpu_data();
  rois_data[0] = 0;
  rois_data[1] = 2.2f;
  rois_data[2] = 2.3f;
  rois_data[3] = 2.6f;
  rois_data[4] = 2.7f;
  rois_data[5] = 0;
  rois_data[6] = 1.8f;
  rois_data[7] = 1.7f;
  rois_data[8] = 4.60f;
  rois_data[9] = 4.75f;
  rois_data[10] = 0;
  rois_data[11] = 2.2f;
  rois_data[12] = 2.3f;
  rois_data[13] = 1.6f;
  rois_data[14] = 1.7f;
  rois_data[15] = 0;
  rois_data[16] = -1.2f;
  rois_data[17] = -2.3f;
  rois_data[18] = 500.6f;
  rois_data[19] = 500.7f;
  std::vector<std::shared_ptr<apollo::perception::base::Blob<float>>> bottoms;
  bottoms.push_back(feat_b);
  bottoms.push_back(rois_b);
  std::vector<std::shared_ptr<apollo::perception::base::Blob<float>>> tops;
  tops.push_back(top);
  roi_layer->ForwardGPU(bottoms, tops);
  ASSERT_EQ(top->cpu_data()[0], 62);
  ASSERT_EQ(top->cpu_data()[48], 62);
  ASSERT_EQ(top->cpu_data()[49], 31);
  ASSERT_EQ(top->cpu_data()[97], 124);
  ASSERT_EQ(top->cpu_data()[98], 62);
  ASSERT_EQ(top->cpu_data()[146], 62);
  ASSERT_EQ(top->cpu_data()[147], 899);
  ASSERT_EQ(top->cpu_data()[195], 0);

  roi_layer->ForwardCPU(bottoms, tops);
  ASSERT_EQ(top->cpu_data()[0], 62);
  ASSERT_EQ(top->cpu_data()[48], 62);
  ASSERT_EQ(top->cpu_data()[49], 31);
  ASSERT_EQ(top->cpu_data()[97], 124);
  ASSERT_EQ(top->cpu_data()[98], 62);
  ASSERT_EQ(top->cpu_data()[146], 62);
  ASSERT_EQ(top->cpu_data()[147], 899);
  ASSERT_EQ(top->cpu_data()[195], 0);
}
TEST(ROIPoolRoundTest, test) {
  int pooled_h = 7;
  int pooled_w = 7;
  bool use_floor = false;
  int feat_channel = 1;
  float spatial_scale = 16.0f;
  auto roi_layer = new apollo::perception::inference::ROIPoolingLayer<float>(
      pooled_h, pooled_w, use_floor, spatial_scale, feat_channel);
  std::vector<int> feat_shape{2, feat_channel, 30, 30};
  std::vector<int> rois_shape{4, 5};
  std::vector<int> top_shape{4, feat_channel, pooled_h, pooled_w};
  std::shared_ptr<apollo::perception::base::Blob<float>> feat_b;
  feat_b.reset(new apollo::perception::base::Blob<float>(feat_shape));
  std::shared_ptr<apollo::perception::base::Blob<float>> rois_b;
  rois_b.reset(new apollo::perception::base::Blob<float>(rois_shape));
  std::shared_ptr<apollo::perception::base::Blob<float>> top;
  top.reset(new apollo::perception::base::Blob<float>(top_shape));
  for (int b = 0; b < 2; b++) {
    for (int c = 0; c < feat_channel; c++) {
      int offset = feat_b->offset(b, c);
      for (int idx = 0; idx < feat_b->shape(2) * feat_b->shape(3); idx++) {
        feat_b->mutable_cpu_data()[offset + idx] = static_cast<float>(idx);
      }
    }
  }

  auto rois_data = rois_b->mutable_cpu_data();
  rois_data[0] = 0;
  rois_data[1] = 2.2f;
  rois_data[2] = 2.3f;
  rois_data[3] = 2.6f;
  rois_data[4] = 2.7f;
  rois_data[5] = 0;
  rois_data[6] = 1.8f;
  rois_data[7] = 1.7f;
  rois_data[8] = 4.60f;
  rois_data[9] = 4.75f;
  rois_data[10] = 0;
  rois_data[11] = 2.2f;
  rois_data[12] = 2.3f;
  rois_data[13] = 1.6f;
  rois_data[14] = 1.7f;
  rois_data[15] = 0;
  rois_data[16] = -1.2f;
  rois_data[17] = -2.3f;
  rois_data[18] = 500.6f;
  rois_data[19] = 500.7f;
  std::vector<std::shared_ptr<apollo::perception::base::Blob<float>>> bottoms;
  bottoms.push_back(feat_b);
  bottoms.push_back(rois_b);
  std::vector<std::shared_ptr<apollo::perception::base::Blob<float>>> tops;
  tops.push_back(top);
  roi_layer->ForwardGPU(bottoms, tops);
  ASSERT_EQ(top->cpu_data()[0], 62);
  ASSERT_EQ(top->cpu_data()[48], 93);
  ASSERT_EQ(top->cpu_data()[49], 62);
  ASSERT_EQ(top->cpu_data()[97], 155);
  ASSERT_EQ(top->cpu_data()[98], 62);
  ASSERT_EQ(top->cpu_data()[146], 62);
  ASSERT_EQ(top->cpu_data()[147], 899);
  ASSERT_EQ(top->cpu_data()[195], 0);

  roi_layer->ForwardCPU(bottoms, tops);
  ASSERT_EQ(top->cpu_data()[0], 62);
  ASSERT_EQ(top->cpu_data()[48], 93);
  ASSERT_EQ(top->cpu_data()[49], 62);
  ASSERT_EQ(top->cpu_data()[97], 155);
  ASSERT_EQ(top->cpu_data()[98], 62);
  ASSERT_EQ(top->cpu_data()[146], 62);
  ASSERT_EQ(top->cpu_data()[147], 899);
  ASSERT_EQ(top->cpu_data()[195], 0);
}
