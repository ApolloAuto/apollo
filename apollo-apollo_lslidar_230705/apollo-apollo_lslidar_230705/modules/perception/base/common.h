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

#pragma once

#include <cassert>

#if USE_GPU == 1
  #if GPU_PLATFORM == NVIDIA
    #include <cublas_v2.h>
    #include <cuda_runtime.h>
    #include <cuda_runtime_api.h>
  #elif GPU_PLATFORM == AMD
    #include <hipblas.h>
    #include <hip/hip_runtime.h>
    #include <hip/hip_runtime_api.h>
    #define cublasCreate hipblasCreate
    #define cublasDestroy hipblasDestroy
    #define cublasHandle_t hipblasHandle_t
    #define cudaDeviceProp hipDeviceProp_t
    #define cudaDeviceSynchronize hipDeviceSynchronize
    #define cudaError_t hipError_t
    #define cudaFree hipFree
    #define cudaFreeHost hipHostFree
    #define cudaGetDevice hipGetDevice
    #define cudaGetDeviceCount hipGetDeviceCount
    #define cudaGetDeviceProperties hipGetDeviceProperties
    #define cudaGetErrorString hipGetErrorString
    #define cudaMalloc hipMalloc
    #define cudaMallocHost hipMallocHost
    #define cudaMemcpy hipMemcpy
    #define cudaMemcpyAsync hipMemcpyAsync
    #define cudaMemcpyDefault hipMemcpyDefault
    #define cudaMemcpyDeviceToDevice hipMemcpyDeviceToDevice
    #define cudaMemcpyDeviceToHost hipMemcpyDeviceToHost
    #define cudaMemcpyHostToDevice hipMemcpyHostToDevice
    #define cudaMemcpyKind hipMemcpyKind
    #define cudaMemset hipMemset
    #define cudaMemsetAsync hipMemsetAsync
    #define cudaPointerAttributes hipPointerAttribute_t
    #define cudaPointerGetAttributes hipPointerGetAttributes
    #define cudaSetDevice hipSetDevice
    #define cudaStream_t hipStream_t
    #define cudaStreamCreate hipStreamCreate
    #define cudaStreamDestroy hipStreamDestroy
    #define cudaStreamSynchronize hipStreamSynchronize
    #define cudaSuccess hipSuccess
  #endif
#endif

namespace apollo {
namespace perception {
namespace base {

#ifndef NO_GPU
#define NO_GPU assert(false)
#endif

#if USE_GPU == 1

#define BASE_GPU_CHECK(condition) \
  { apollo::perception::base::GPUAssert((condition), __FILE__, __LINE__); }

inline void GPUAssert(cudaError_t code, const char *file, int line,
                      bool abort = true) {
  if (code != cudaSuccess) {
    fprintf(stderr, "GPUassert: %s %s %d\n", cudaGetErrorString(code), file,
            line);
    if (abort) {
      exit(code);
    }
  }
}

#endif

}  // namespace base
}  // namespace perception
}  // namespace apollo
