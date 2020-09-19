/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include <cuda_runtime.h>

namespace apollo {
namespace planning {

#define BLOCK_WIDTH 16
#define BLOCK_HEIGHT 16

#define BLOCK_1 256

#define TEMPLATE_ROUTINE_INSTANCE(ret, routine) template ret routine

#define DATA_TRANSFER_INST(type) \
  TEMPLATE_ROUTINE_INSTANCE(     \
      bool, data_transfer(type *dst, const type *src, const int size))

#define CUDA_CHECK(call)                                                   \
  {                                                                        \
    const cudaError_t error = call;                                        \
    if (error != cudaSuccess) {                                            \
      printf("Error: %s:%d, ", __FILE__, __LINE__);                        \
      printf("code: %d, reasone: %s\n", error, cudaGetErrorString(error)); \
      return false;                                                        \
    }                                                                      \
  }

bool InitialCuda();

__global__ void fill_lower_left_gpu(int *iRow, int *jCol, unsigned int *rind_L,
                                    unsigned int *cind_L, const int nnz_L);

template <typename T>
__global__ void data_transfer_gpu(T *dst, const T *src, const int size);

bool fill_lower_left(int *iRow, int *jCol, unsigned int *rind_L,
                     unsigned int *cind_L, const int nnz_L);
template <typename T>
bool data_transfer(T *dst, const T *src, const int size);

}  // namespace planning
}  // namespace apollo
