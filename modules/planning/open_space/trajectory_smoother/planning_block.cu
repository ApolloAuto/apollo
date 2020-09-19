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

#include <iostream>

#include "planning_block.h"

namespace apollo {
namespace planning {
bool InitialCuda() {
  int dev = 0;
  cudaDeviceProp deviceProp;
  CUDA_CHECK(cudaGetDeviceProperties(&deviceProp, dev));
  CUDA_CHECK(cudaSetDevice(dev));
  return true;
}

__global__ void fill_lower_left_gpu(int *iRow, int *jCol, unsigned int *rind_L,
                                    unsigned int *cind_L, const int nnz_L) {
  int i = threadIdx.x;

  if (i < nnz_L) {
    iRow[i] = rind_L[i];
    jCol[i] = cind_L[i];
  }
}

template <typename T>
__global__ void data_transfer_gpu(T *dst, const T *src, const int size) {
  int i = threadIdx.x;

  if (i < size) {
    dst[i] = src[i];
  }
}

bool fill_lower_left(int *iRow, int *jCol, unsigned int *rind_L,
                     unsigned int *cind_L, const int nnz_L) {
  if (!InitialCuda()) return false;
  int *d_iRow, *d_jCol;
  unsigned int *d_rind_L, *d_cind_L;

  unsigned int nBytes = nnz_L * sizeof(int);
  unsigned int nUBytes = nnz_L * sizeof(unsigned int);
  cudaMalloc((void **)&d_iRow, nBytes);
  cudaMalloc((void **)&d_jCol, nBytes);
  cudaMalloc((void **)&d_rind_L, nUBytes);
  cudaMalloc((void **)&d_cind_L, nUBytes);

  cudaMemcpy(d_iRow, iRow, nBytes, cudaMemcpyHostToDevice);
  cudaMemcpy(d_jCol, jCol, nBytes, cudaMemcpyHostToDevice);

  dim3 block(BLOCK_1);
  dim3 grid((nnz_L + block.x - 1) / block.x);

  fill_lower_left_gpu<<<grid, block>>>(d_iRow, d_jCol, d_rind_L, d_cind_L,
                                       nnz_L);
  cudaDeviceSynchronize();

  cudaMemcpy(rind_L, d_rind_L, nUBytes, cudaMemcpyDeviceToHost);
  cudaMemcpy(cind_L, d_cind_L, nUBytes, cudaMemcpyDeviceToHost);

  cudaFree(d_iRow);
  cudaFree(d_jCol);
  cudaFree(d_rind_L);
  cudaFree(d_cind_L);
  cudaDeviceReset();
  return true;
}

template <typename T>
bool data_transfer(T *dst, const T *src, const int size) {
  if (!InitialCuda()) return false;
  T *d_dst, *d_src;
  size_t nBytes = size * sizeof(T);
  cudaMalloc((void **)&d_dst, nBytes);
  cudaMalloc((void **)&d_src, nBytes);
  cudaMemcpy(d_src, src, nBytes, cudaMemcpyHostToDevice);
  cudaMemcpy(d_dst, dst, nBytes, cudaMemcpyHostToDevice);

  dim3 block(BLOCK_1);
  dim3 grid((size + block.x - 1) / block.x);

  data_transfer_gpu<<<grid, block>>>(dst, src, size);
  cudaDeviceSynchronize();

  cudaMemcpy(dst, d_dst, nBytes, cudaMemcpyDeviceToHost);

  cudaFree(d_dst);
  cudaFree(d_src);
  cudaDeviceReset();
  return true;
}

DATA_TRANSFER_INST(int);
DATA_TRANSFER_INST(double);
DATA_TRANSFER_INST(float);

}  // namespace planning
}  // namespace apollo
