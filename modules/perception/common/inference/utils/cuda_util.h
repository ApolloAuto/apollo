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

#if GPU_PLATFORM == NVIDIA
  #include <cublas_v2.h>
  #include <cuda_runtime_api.h>
#elif GPU_PLATFORM == AMD
  #include <hipblas.h>
  #include <hip/hip_runtime_api.h>
  #define CUBLAS_STATUS_SUCCESS HIPBLAS_STATUS_SUCCESS
  #define CUBLAS_OP_N HIPBLAS_OP_N
  #define CUBLAS_OP_T HIPBLAS_OP_T
  #define cublasCreate hipblasCreate
  #define cublasDestroy hipblasDestroy
  #define cublasHandle_t hipblasHandle_t
  #define cublasOperation_t hipblasOperation_t
  #define cublasSgemm hipblasSgemm
  #define cublasStatus_t hipblasStatus_t
  #define cudaGetDevice hipGetDevice
  #define cudaGetErrorString hipGetErrorString
  #define cudaSetDevice hipSetDevice
  #define cudaSuccess hipSuccess
#endif

namespace apollo {
namespace perception {
namespace inference {

class CudaUtil {
 public:
  static bool set_device_id(int device_id);
  static cublasHandle_t& get_handler();
  ~CudaUtil();

 private:
  CudaUtil();
  static CudaUtil& get();
  cublasHandle_t cublas_handle_;
};

}  // namespace inference
}  // namespace perception
}  // namespace apollo
