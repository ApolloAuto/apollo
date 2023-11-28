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

#include "modules/perception/common/inference/utils/cuda_util.h"

#include <boost/thread.hpp>

#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace inference {

static boost::thread_specific_ptr<CudaUtil> thread_instance_;

#define CUBLAS_CHECK(condition)                               \
  do {                                                        \
    cublasStatus_t status = condition;                        \
    CHECK_EQ(status, CUBLAS_STATUS_SUCCESS) << " " << status; \
  } while (0)

CudaUtil &CudaUtil::get() {
  if (!thread_instance_.get()) {
    thread_instance_.reset(new CudaUtil);
  }
  return *(thread_instance_.get());
}

CudaUtil::CudaUtil() { CUBLAS_CHECK(cublasCreate(&cublas_handle_)); }

bool CudaUtil::set_device_id(int device_id) {
  int now_device = -1;
  auto cuda_error = cudaGetDevice(&now_device);
  CHECK_EQ(cuda_error, cudaSuccess) << " " << cudaGetErrorString(cuda_error);
  if (now_device == device_id) {
    return true;
  } else {
    cuda_error = cudaSetDevice(device_id);
    CHECK_EQ(cuda_error, cudaSuccess) << " " << cudaGetErrorString(cuda_error);
    if (get().cublas_handle_ != nullptr) {
      CUBLAS_CHECK(cublasDestroy(get().cublas_handle_));
    }

    CUBLAS_CHECK(cublasCreate(&get().cublas_handle_));
  }
  return true;
}
cublasHandle_t &CudaUtil::get_handler() { return get().cublas_handle_; }

CudaUtil::~CudaUtil() {
  if (get().cublas_handle_) {
    CUBLAS_CHECK(cublasDestroy(get().cublas_handle_));
  }
}

}  // namespace inference
}  // namespace perception
}  // namespace apollo
