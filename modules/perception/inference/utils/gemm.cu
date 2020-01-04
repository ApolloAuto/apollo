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

#include "modules/perception/inference/utils/gemm.h"

#include <algorithm>

#include "modules/perception/inference/utils/cuda_util.h"
#include "modules/perception/inference/utils/util.h"

namespace apollo {
namespace perception {
namespace inference {

__global__ void sqrt_kernel(float *data, int width, int height) {
  const int x = blockDim.x * blockIdx.x + threadIdx.x;
  const int y = blockDim.y * blockIdx.y + threadIdx.y;

  if (x < width && y < height) {
    int index = y * width + x;
    if (data[index] > 0) {
      data[index] = 1.0f / static_cast<float>(sqrt(data[index]));
    } else {
      float dim = width * height;
      data[index] = 1.0f / static_cast<float>(sqrt(dim));
    }
  }
}
__global__ void mul_kernel(const int n, const float *a, const float *b,
                           float *y) {
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < (n);
       i += blockDim.x * gridDim.x) {
    y[i] = a[i] * b[i];
  }
}
__global__ void multi_scale_kernel(const float *data_in, const float *scale,
                                   float *data_out, int width, int height) {
  const int x = blockDim.x * blockIdx.x + threadIdx.x;
  const int y = blockDim.y * blockIdx.y + threadIdx.y;

  if (x < width && y < height) {
    int index = y * width + x;
    data_out[index] = data_in[index] * scale[y];
  }
}
__global__ void set_kernel(const int n, const float alpha, float *y) {
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < (n);
       i += blockDim.x * gridDim.x) {
    y[i] = alpha;
  }
}

void GPUL2Norm::L2Norm(base::Blob<float> *input_data) {
  int num = input_data->num();
  if (num == 0) {
    return;
  }
  int dim = input_data->count() / num;
  square_.Reshape({num, dim});
  ones_.Reshape({dim, 1});
  scale_.Reshape({num, 1});

  GPUMSetFloat(ones_.count(), 1, ones_.mutable_gpu_data());

  // x = input^2
  GPUMultiFloat(input_data->count(), input_data->gpu_data(),
                input_data->gpu_data(), square_.mutable_gpu_data());
  // scale_ = (numxdim)*(dimx1) = (numx1)
  GPUGemmFloat(CblasNoTrans, CblasTrans, num, 1, dim, 1.0, square_.gpu_data(),
               ones_.gpu_data(), 0.0, scale_.mutable_gpu_data());
  dim3 threadsPerBlock(32, 8);
  dim3 numBlocks(dim / threadsPerBlock.x + 1, num / threadsPerBlock.y + 1);
  sqrt_kernel<<<numBlocks, threadsPerBlock>>>(scale_.mutable_gpu_data(), 1,
                                              num);

  multi_scale_kernel<<<numBlocks, threadsPerBlock>>>(
      input_data->gpu_data(), scale_.gpu_data(), input_data->mutable_gpu_data(),
      dim, num);
}

void GPUMultiFloat(const int n, const float *a, const float *b, float *result) {
  const int CUDA_THREAD = 512;
  mul_kernel<<<(n + CUDA_THREAD - 1) / CUDA_THREAD, CUDA_THREAD>>>(n, a, b,
                                                                   result);
}
void GPUMSetFloat(const int n, const float alpha, float *result) {
  const int CUDA_THREAD = 512;
  set_kernel<<<(n + CUDA_THREAD - 1) / CUDA_THREAD, CUDA_THREAD>>>(n, alpha,
                                                                   result);
}
void GPUGemmFloat(const CBLAS_TRANSPOSE TransA, const CBLAS_TRANSPOSE TransB,
                  const int M, const int N, const int K, const float alpha,
                  const float *A, const float *B, const float beta, float *C) {
  // Note that cublas follows fortran order.
  int lda = (TransA == CblasNoTrans) ? K : M;
  int ldb = (TransB == CblasNoTrans) ? N : K;
  cublasOperation_t cuTransA =
      (TransA == CblasNoTrans) ? CUBLAS_OP_N : CUBLAS_OP_T;
  cublasOperation_t cuTransB =
      (TransB == CblasNoTrans) ? CUBLAS_OP_N : CUBLAS_OP_T;
  CHECK(cublasSgemm(CudaUtil::get_handler(), cuTransB, cuTransA, N, M, K,
                    &alpha, B, ldb, A, lda, &beta, C,
                    N) == CUBLAS_STATUS_SUCCESS);
}

}  // namespace inference
}  // namespace perception
}  // namespace apollo
