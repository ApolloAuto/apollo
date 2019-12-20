/******************************************************************************
COPYRIGHT

All contributions by the University of California:
Copyright (c) 2014-2017 The Regents of the University of California (Regents)
All rights reserved.

All other contributions:
Copyright (c) 2014-2017, the respective contributors
All rights reserved.

Caffe uses a shared copyright model: each contributor holds copyright over
their contributions to Caffe. The project versioning records all such
contribution and copyright details. If a contributor wants to further mark
their specific copyright on a particular contribution, they should indicate
their copyright solely in the commit message of the change when it is
committed.

LICENSE

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

CONTRIBUTION AGREEMENT

By contributing to the BVLC/caffe repository through pull-request, comment,
or otherwise, the contributor releases their content to the
license and copyright terms herein.
 *****************************************************************************/

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

#include "modules/perception/base/blob.h"

#include "gtest/gtest.h"

#include "modules/perception/base/test/test_helper.h"

namespace apollo {
namespace perception {
namespace base {

TEST(BlobTest, header_test) {
  Blob<float> blob_empty;
  EXPECT_EQ(blob_empty.shape_string(), "(0)");
  EXPECT_EQ(blob_empty.count(), 0);

  Blob<float> blob(1, 2, 3, 4);
  Blob<float> blob_2d({1, 2});

  // test shape_string
  EXPECT_EQ(blob.shape_string(), "1 2 3 4 (24)");
  // test count
  EXPECT_EQ(blob.count(), 24);
  EXPECT_EQ(blob.count(0), 24);
  EXPECT_EQ(blob.count(0, 4), 24);
  EXPECT_EQ(blob.count(0, 0), 1);
  EXPECT_EQ(blob.count(1, 3), 6);
  // test CanonicalAxisIndex
  EXPECT_EQ(blob.CanonicalAxisIndex(3), 3);
  EXPECT_EQ(blob.CanonicalAxisIndex(-1), 3);
  // test LegacyShape
  EXPECT_EQ(blob.LegacyShape(0), 1);
  EXPECT_EQ(blob.LegacyShape(1), 2);
  EXPECT_EQ(blob.LegacyShape(2), 3);
  EXPECT_EQ(blob.LegacyShape(3), 4);
  EXPECT_EQ(blob_2d.LegacyShape(3), 1);
  EXPECT_EQ(blob_2d.LegacyShape(-3), 1);
  // test offset
  EXPECT_EQ(blob.offset({0, 0, 0, 0}), 0);
  EXPECT_EQ(blob.offset({0, 0, 0, 1}), 1);
  EXPECT_EQ(blob.offset({0, 1, 0, 0}), 12);
}

TEST(BlobTest, source_test) {
  Blob<float> blob_4d(1, 2, 3, 4);
  Blob<float> blob_2d;
  blob_2d.Reshape({2, 3});
  blob_4d.Reshape({1, 1, 6, 4});
  blob_4d.Reshape({2, 3, 4, 5});

  float cpu_data[120];
  blob_4d.set_cpu_data(cpu_data);
  // decrease count_ and makke count_ * sizeof(Dtype) < data_.size()
  blob_4d.Reshape({1, 3, 4, 5});
  blob_4d.set_cpu_data(cpu_data);

#ifndef PERCEPTION_CPU_ONLY
  blob_4d.Reshape({2, 3, 4, 5});
  float* gpu_data = nullptr;
  BASE_CUDA_CHECK(cudaMalloc(&gpu_data, blob_4d.count() * sizeof(float)));
  blob_4d.set_gpu_data(gpu_data);
  blob_4d.Reshape({1, 3, 4, 5});
  blob_4d.set_gpu_data(gpu_data);
#endif
}

template <typename Dtype>
class BlobSimpleTest : public ::testing::Test {
 protected:
  BlobSimpleTest()
      : blob_(new Blob<Dtype>()),
        blob_preshaped_(new Blob<Dtype>(2, 3, 4, 5)) {}
  virtual ~BlobSimpleTest() {
    delete blob_;
    delete blob_preshaped_;
  }
  Blob<Dtype>* const blob_;
  Blob<Dtype>* const blob_preshaped_;
};

TYPED_TEST_CASE(BlobSimpleTest, TestDtypes);

TYPED_TEST(BlobSimpleTest, TestInitialization) {
  EXPECT_TRUE(this->blob_);
  EXPECT_TRUE(this->blob_preshaped_);
  EXPECT_EQ(this->blob_preshaped_->num(), 2);
  EXPECT_EQ(this->blob_preshaped_->channels(), 3);
  EXPECT_EQ(this->blob_preshaped_->height(), 4);
  EXPECT_EQ(this->blob_preshaped_->width(), 5);
  EXPECT_EQ(this->blob_preshaped_->count(), 120);
  EXPECT_EQ(this->blob_->num_axes(), 0);
  EXPECT_EQ(this->blob_->count(), 0);
}

TYPED_TEST(BlobSimpleTest, TestPointersCPUGPU) {
  EXPECT_TRUE(this->blob_preshaped_->gpu_data());
  EXPECT_TRUE(this->blob_preshaped_->cpu_data());
  EXPECT_TRUE(this->blob_preshaped_->mutable_gpu_data());
  EXPECT_TRUE(this->blob_preshaped_->mutable_cpu_data());
}

TYPED_TEST(BlobSimpleTest, TestReshape) {
  this->blob_->Reshape(2, 3, 4, 5);
  EXPECT_EQ(this->blob_->num(), 2);
  EXPECT_EQ(this->blob_->channels(), 3);
  EXPECT_EQ(this->blob_->height(), 4);
  EXPECT_EQ(this->blob_->width(), 5);
  EXPECT_EQ(this->blob_->count(), 120);
}

TYPED_TEST(BlobSimpleTest, TestReshapeZero) {
  std::vector<int> shape(2);
  shape[0] = 0;
  shape[1] = 5;
  this->blob_->Reshape(shape);
  EXPECT_EQ(this->blob_->count(), 0);
}

#if 0
TYPED_TEST(BlobSimpleTest, TestLegacyBlobProtoShapeEquals) {
  BlobProto blob_proto;

  // Reshape to (3 x 2).
  std::vector<int> shape(2);
  shape[0] = 3;
  shape[1] = 2;
  this->blob_->Reshape(shape);

  // (3 x 2) blob == (1 x 1 x 3 x 2) legacy blob
  blob_proto.set_num(1);
  blob_proto.set_channels(1);
  blob_proto.set_height(3);
  blob_proto.set_width(2);
  EXPECT_TRUE(this->blob_->ShapeEquals(blob_proto));

  // (3 x 2) blob != (0 x 1 x 3 x 2) legacy blob
  blob_proto.set_num(0);
  blob_proto.set_channels(1);
  blob_proto.set_height(3);
  blob_proto.set_width(2);
  EXPECT_FALSE(this->blob_->ShapeEquals(blob_proto));

  // (3 x 2) blob != (3 x 1 x 3 x 2) legacy blob
  blob_proto.set_num(3);
  blob_proto.set_channels(1);
  blob_proto.set_height(3);
  blob_proto.set_width(2);
  EXPECT_FALSE(this->blob_->ShapeEquals(blob_proto));

  // Reshape to (1 x 3 x 2).
  shape.insert(shape.begin(), 1);
  this->blob_->Reshape(shape);

  // (1 x 3 x 2) blob == (1 x 1 x 3 x 2) legacy blob
  blob_proto.set_num(1);
  blob_proto.set_channels(1);
  blob_proto.set_height(3);
  blob_proto.set_width(2);
  EXPECT_TRUE(this->blob_->ShapeEquals(blob_proto));

  // Reshape to (2 x 3 x 2).
  shape[0] = 2;
  this->blob_->Reshape(shape);

  // (2 x 3 x 2) blob != (1 x 1 x 3 x 2) legacy blob
  blob_proto.set_num(1);
  blob_proto.set_channels(1);
  blob_proto.set_height(3);
  blob_proto.set_width(2);
  EXPECT_FALSE(this->blob_->ShapeEquals(blob_proto));
}
#endif

template <typename TypeParam>
class BlobMathTest : public MultiDeviceTest<TypeParam> {
  typedef typename TypeParam::Dtype Dtype;

 protected:
  BlobMathTest() : blob_(new Blob<Dtype>(2, 3, 4, 5)), epsilon_(1e-6) {}

  virtual ~BlobMathTest() { delete blob_; }
  Blob<Dtype>* const blob_;
  Dtype epsilon_;
};

TYPED_TEST_CASE(BlobMathTest, TestDtypesAndDevices);

}  // namespace base
}  // namespace perception
}  // namespace apollo
