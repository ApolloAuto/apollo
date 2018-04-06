/******************************************************************************
 * Copyright (c) 2014, Victor Matheus de Araujo Oliveira All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

#include <iostream>

#include "block_uf.h"
#include "texture.h"

namespace apollo {
namespace perception {
namespace block_uf {

__device__ int Find(int* parent, int i) {
  while (parent[i] != i) {
    i = parent[i];
  }
  return i;
}

__device__ void Union(int* parent, int i, int j) {
  bool done;
  do {
    i = Find(parent, i);
    j = Find(parent, j);

    if (i < j) {
      int old = atomicMin(&parent[j], i);
      done = (old == j);
      j = old;
    } else if (i > j) {
      int old = atomicMin(&parent[i], j);
      done = (old == i);
      i = old;
    } else {
      done = true;
    }
  } while (!done);
}

__global__ void BlockUnionFindInternal(int* label_array, int width,
                                       int height) {
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;
  int global_index = y * width + x;
  int block_index = blockDim.x * threadIdx.y + threadIdx.x;

  __shared__ int s_parent[UF_BLOCK_WIDTH * UF_BLOCK_HEIGHT];
  __shared__ unsigned char s_img[UF_BLOCK_WIDTH * UF_BLOCK_HEIGHT];

  bool is_valid = x < width && y < height;

  s_parent[block_index] = block_index;
  s_img[block_index] = is_valid ? tex2D(img_tex, x, y) : 0xFF;
  __syncthreads();

  unsigned char v = s_img[block_index];

  if (is_valid && threadIdx.x > 0 && v != 0 && s_img[block_index - 1] == v) {
    Union(s_parent, block_index, block_index - 1);
  }
  __syncthreads();

  if (is_valid && threadIdx.y > 0 && v != 0 &&
      s_img[block_index - blockDim.x] == v) {
    Union(s_parent, block_index, block_index - blockDim.x);
  }
  __syncthreads();

  if (is_valid) {
    int f = Find(s_parent, block_index);
    int fx = f % UF_BLOCK_WIDTH;
    int fy = f / UF_BLOCK_WIDTH;
    label_array[global_index] =
        (blockIdx.y * blockDim.y + fy) * width +
            (blockIdx.x * blockDim.x + fx);
  }
}

__global__ void BlockUnionFindBoundary(int* label_array, int width,
                                       int height) {
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;
  int global_index = y * width + x;

  bool is_valid = x < width && y < height;
  unsigned char v = is_valid ? tex2D(img_tex, x, y) : 0xFF;

  if (is_valid && y > 0 && threadIdx.y == 0 && v != 0 &&
      tex2D(img_tex, x, y - 1) == v) {
    Union(label_array, global_index, global_index - width);
  }

  if (is_valid && x > 0 && threadIdx.x == 0 && v != 0 &&
      tex2D(img_tex, x - 1, y) == v) {
    Union(label_array, global_index, global_index - 1);
  }
}

__global__ void BlockUnionFindRoot(int* label_array, int width, int height) {
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;
  int global_index = y * width + x;

  bool is_valid = x < width && y < height;
  if (is_valid) {
    label_array[global_index] =
        tex2D(img_tex, x, y) > 0 ? Find(label_array, global_index) : -1;
  }
}

bool BlockUnionFind(const unsigned char* img, int width, int height,
                    int image_width, int* labels) {
  cudaError_t cuda_err;

  cudaChannelFormatDesc uchar_desc = cudaCreateChannelDesc<unsigned char>();
  cudaArray* img_array = NULL;
  cudaMallocArray(&img_array, &uchar_desc, static_cast<size_t>(width),
                  static_cast<size_t>(height));
  cudaBindTextureToArray(img_tex, img_array, uchar_desc);

  if (image_width == width) {
    size_t siz = static_cast<size_t>(width) * static_cast<size_t>(height) *
                 sizeof(unsigned char);
    cudaMemcpyToArray(img_array, 0, 0, img, siz, cudaMemcpyHostToDevice);
  } else {
    size_t siz = static_cast<size_t>(width) * sizeof(unsigned char);
    for (size_t i = 0; i < static_cast<size_t>(height); ++i) {
      cudaMemcpyToArray(img_array, 0, i, img, siz, cudaMemcpyHostToDevice);
      img += image_width;
    }
  }

  int* label_array;
  cudaMalloc(
      (void**)&label_array,
      static_cast<size_t>(width) * static_cast<size_t>(height) * sizeof(int));

  dim3 block(UF_BLOCK_WIDTH, UF_BLOCK_HEIGHT);
  dim3 grid(
      static_cast<unsigned int>((width + UF_BLOCK_WIDTH - 1) / UF_BLOCK_WIDTH),
      static_cast<unsigned int>((height + UF_BLOCK_HEIGHT - 1) /
                                UF_BLOCK_HEIGHT));
  cuda_err = cudaGetLastError();
  if (cuda_err != cudaSuccess) {
    std::cerr << "failed to start block union find with CUDA: "
              << cudaGetErrorString(cuda_err) << std::endl;
    return false;
  }

  cudaThreadSetCacheConfig(cudaFuncCachePreferShared);

  BlockUnionFindInternal<<<grid, block>>>(label_array, width, height);

  cudaThreadSetCacheConfig(cudaFuncCachePreferL1);

  BlockUnionFindBoundary<<<grid, block>>>(label_array, width, height);

  BlockUnionFindRoot<<<grid, block>>>(label_array, width, height);

  cudaMemcpy(labels, label_array,
             static_cast<size_t>(width) *
                 static_cast<size_t>(height) * sizeof(int),
             cudaMemcpyDeviceToHost);

  cudaFree(label_array);
  cudaFreeArray(img_array);
  cuda_err = cudaGetLastError();
  if (cuda_err != cudaSuccess) {
    std::cerr << "failed to end block union find with CUDA: "
              << cudaGetErrorString(cuda_err) << std::endl;
    return false;
  }

  return true;
}

}  // namespace block_uf
}  // namespace perception
}  // namespace apollo
