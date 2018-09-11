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
#include "cybertron/croutine/system/cuda_async.h"

#include "cybertron/croutine/croutine.h"

namespace apollo {
namespace cybertron {
namespace croutine {

void CUDART_CB CudaAsyncCallback(cudaStream_t event, cudaError_t status,
                                 void* data) {
  auto routine = static_cast<CRoutine*>(data);
  routine->Wake();
}

void CudaAsync(const cudaStream_t& stream) {
  auto routine = CRoutine::GetCurrentRoutine();
  cudaStreamAddCallback(stream, CudaAsyncCallback, (void*)routine, 0);
  routine->HangUp();
}

}  // namespace croutine
}  // namespace cybertron
}  // namespace apollo
