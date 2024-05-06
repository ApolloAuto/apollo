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

#include "modules/perception/common/base/syncedmem.h"

#include "gtest/gtest.h"

#include "modules/perception/common/base/test/test_helper.h"

namespace apollo {
namespace perception {
namespace base {

class SyncedMemoryTest : public ::testing::Test {};

TEST_F(SyncedMemoryTest, header_test) {
  {
    bool use_cuda = true;
    void* ptr;
    PerceptionMallocHost(&ptr, 100, use_cuda);
    EXPECT_TRUE(ptr);
    PerceptionFreeHost(ptr, use_cuda);
  }

  {
    bool use_cuda = false;
    void* ptr;
    PerceptionMallocHost(&ptr, 100, use_cuda);
    EXPECT_TRUE(ptr);
    PerceptionFreeHost(ptr, use_cuda);
  }
}

TEST_F(SyncedMemoryTest, TestInitialization) {
  SyncedMemory mem(10, true);
  EXPECT_EQ(mem.head(), SyncedMemory::UNINITIALIZED);
  EXPECT_EQ(mem.size(), 10);
  SyncedMemory* p_mem = new SyncedMemory(10 * sizeof(float), true);
  EXPECT_EQ(p_mem->size(), 10 * sizeof(float));
  delete p_mem;
}

#if USE_GPU == 1  // GPU test

TEST_F(SyncedMemoryTest, TestAllocationCPUGPU) {
  SyncedMemory mem(10, true);
  EXPECT_TRUE(mem.cpu_data());
  EXPECT_TRUE(mem.gpu_data());
  EXPECT_TRUE(mem.mutable_cpu_data());
  EXPECT_TRUE(mem.mutable_gpu_data());
}

#endif

TEST_F(SyncedMemoryTest, TestAllocationCPU) {
  SyncedMemory mem(10, true);
  EXPECT_TRUE(mem.cpu_data());
  EXPECT_TRUE(mem.mutable_cpu_data());
}

#if USE_GPU == 1  // GPU test

TEST_F(SyncedMemoryTest, TestAllocationGPU) {
  SyncedMemory mem(10, true);
  EXPECT_TRUE(mem.gpu_data());
  EXPECT_TRUE(mem.mutable_gpu_data());
}

#endif

TEST_F(SyncedMemoryTest, TestCPUWrite) {
  SyncedMemory mem(10, true);
  void* cpu_data = mem.mutable_cpu_data();
  EXPECT_EQ(mem.head(), SyncedMemory::HEAD_AT_CPU);
  memset(cpu_data, 1, mem.size());
  for (size_t i = 0; i < mem.size(); ++i) {
    EXPECT_EQ((static_cast<char*>(cpu_data))[i], 1);
  }
  // do another round
  cpu_data = mem.mutable_cpu_data();
  EXPECT_EQ(mem.head(), SyncedMemory::HEAD_AT_CPU);
  memset(cpu_data, 2, mem.size());
  for (size_t i = 0; i < mem.size(); ++i) {
    EXPECT_EQ((static_cast<char*>(cpu_data))[i], 2);
  }
}

#if USE_GPU == 1  // GPU test

TEST_F(SyncedMemoryTest, TestGPURead) {
  SyncedMemory mem(10, true);
  void* cpu_data = mem.mutable_cpu_data();
  EXPECT_EQ(mem.head(), SyncedMemory::HEAD_AT_CPU);
  memset(cpu_data, 1, mem.size());
  const void* gpu_data = mem.gpu_data();
  EXPECT_EQ(mem.head(), SyncedMemory::SYNCED);
  // check if values are the same
  char* recovered_value = new char[10];
  cudaMemcpy(recovered_value, gpu_data, 10, cudaMemcpyDefault);
  for (size_t i = 0; i < mem.size(); ++i) {
    EXPECT_EQ((static_cast<char*>(recovered_value))[i], 1);
  }
  // do another round
  cpu_data = mem.mutable_cpu_data();
  EXPECT_EQ(mem.head(), SyncedMemory::HEAD_AT_CPU);
  memset(cpu_data, 2, mem.size());
  for (size_t i = 0; i < mem.size(); ++i) {
    EXPECT_EQ((static_cast<char*>(cpu_data))[i], 2);
  }
  gpu_data = mem.gpu_data();
  EXPECT_EQ(mem.head(), SyncedMemory::SYNCED);
  // check if values are the same
  cudaMemcpy(recovered_value, gpu_data, 10, cudaMemcpyDefault);
  for (size_t i = 0; i < mem.size(); ++i) {
    EXPECT_EQ((static_cast<char*>(recovered_value))[i], 2);
  }
  delete[] recovered_value;
}

TEST_F(SyncedMemoryTest, TestGPUWrite) {
  SyncedMemory mem(10, true);
  void* gpu_data = mem.mutable_gpu_data();
  EXPECT_EQ(mem.head(), SyncedMemory::HEAD_AT_GPU);
  cudaMemset(gpu_data, 1, mem.size());
  const void* cpu_data = mem.cpu_data();
  EXPECT_NE(cpu_data, nullptr);
  for (size_t i = 0; i < mem.size(); ++i) {
    EXPECT_EQ((static_cast<const char*>(cpu_data))[i], 1);
  }
  EXPECT_EQ(mem.head(), SyncedMemory::SYNCED);

  gpu_data = mem.mutable_gpu_data();
  EXPECT_EQ(mem.head(), SyncedMemory::HEAD_AT_GPU);
  cudaMemset(gpu_data, 2, mem.size());
  cpu_data = mem.cpu_data();
  EXPECT_NE(cpu_data, nullptr);
  for (size_t i = 0; i < mem.size(); ++i) {
    EXPECT_EQ((static_cast<const char*>(cpu_data))[i], 2);
  }
  EXPECT_EQ(mem.head(), SyncedMemory::SYNCED);
}

#endif

}  // namespace base
}  // namespace perception
}  // namespace apollo
