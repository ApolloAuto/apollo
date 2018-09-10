/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "examples/component/perception.h"

#include "cuda_runtime.h"
#include "cybertron/common/log.h"
#include "cybertron/croutine/system/cuda_async.h"

__global__ void
VectorAdd(const float *A, const float *B, float *C, int numElements) {
  int i = blockDim.x * blockIdx.x + threadIdx.x;
  if (i < numElements) {
    for (int j = 0; j < 1000; ++j) {
      C[i] = A[i] + B[i];
    }
  }
}

int TestCUDA() {
  if (cudaSetDevice(0) != cudaSuccess) {
    return -1;
  }
  //cudaError_t err = cudaSuccess;
  int numElements = 10000000;
  size_t size = numElements * sizeof(float);

  float *h_A = (float *)malloc(size);
  float *h_B = (float *)malloc(size);
  float *h_C = (float *)malloc(size);
  for (int i = 0; i < numElements; ++i) {
      h_A[i] = rand()/(float)RAND_MAX;
      h_B[i] = rand()/(float)RAND_MAX;
  }

  float *d_A = NULL;
  cudaMalloc((void **)&d_A, size);
  float *d_B = NULL;
  cudaMalloc((void **)&d_B, size);
  float *d_C = NULL;
  cudaMalloc((void **)&d_C, size);

  cudaMemcpy(d_A, h_A, size, cudaMemcpyHostToDevice);
  cudaMemcpy(d_B, h_B, size, cudaMemcpyHostToDevice);

  cudaStream_t stream;
  cudaStreamCreate(&stream);

  int threadsPerBlock = 256;
  int blocksPerGrid = (numElements + threadsPerBlock - 1) / threadsPerBlock;
  VectorAdd<<<blocksPerGrid, threadsPerBlock, 0, stream>>>(d_A, d_B, d_C, numElements);
  apollo::cybertron::croutine::CudaAsync(stream);
  cudaMemcpy(h_C, d_C, size, cudaMemcpyDeviceToHost);
  for (int i = 0; i < numElements; ++i) {
    if (fabs(h_A[i] + h_B[i] - h_C[i]) > 1e-5) {
        AERROR << "Result verification failed at element " <<  i;
        return -1;
    }
  }

  cudaFree(d_A);
  cudaFree(d_B);
  cudaFree(d_C);
  free(h_A);
  free(h_B);
  free(h_C);
  return 0;
}
