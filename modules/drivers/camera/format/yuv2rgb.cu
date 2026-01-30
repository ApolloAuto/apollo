/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include <cuda_runtime.h>
#include "modules/drivers/camera/format/yuv2rgb.h"

namespace apollo {
namespace drivers {
namespace camera {

__host__ CudaConvertHandler::~CudaConvertHandler() {}

__host__ CudaConvertHandler::CudaConvertHandler(
  unsigned int width, unsigned int height) : width_(width), height_(height) {}

__device__ inline float clamp(const int val, const int min, const int max)
{
	return (val >= min)? ((val <= max)? val : max) : min;
}

__global__ void kernel(
			unsigned char* src, unsigned char* dst,
      unsigned int width, unsigned int width_factor) {
	int i = blockIdx.x * blockDim.x + threadIdx.x;
  int j = blockIdx.y * blockDim.y + threadIdx.y;
  int pixel_idx = i * 2;
  if (pixel_idx >= width) {
		return;
	}

	int y0 = src[j*width*width_factor+pixel_idx*2+0];
	int cb = src[j*width*width_factor+pixel_idx*2+1];
	int y1 = src[j*width*width_factor+pixel_idx*2+2];
	int cr = src[j*width*width_factor+pixel_idx*2+3];

  // magic numbers based on ITUR BT601 Full Range, YUV => RGB
	dst[j*width*3+pixel_idx*3+0] = clamp(
		(75 * y0 + 102 * cr - 14266 + 32) >> 6, 0, 255);
	dst[j*width*3+pixel_idx*3+1] = clamp(
		(75 * y0 - 52 * cr - 25 * cb + 8671 + 32) >> 6, 0, 255);
	dst[j*width*3+pixel_idx*3+2] = clamp(
		(75 * y0 + 129 * cb - 17723 + 32) >> 6, 0, 255);

	dst[j*width*3+pixel_idx*3+3] = clamp(
		(75 * y1 + 102 * cr - 14266 + 32) >> 6, 0, 255);
	dst[j*width*3+pixel_idx*3+4] = clamp(
		(75 * y1 - 52 * cr - 25 * cb + 8671 + 32) >> 6, 0, 255);
	dst[j*width*3+pixel_idx*3+5] = clamp(
		(75 * y1 + 129 * cb - 17723 + 32) >> 6,	0, 255);
}

__host__ bool CudaConvertHandler::Init() {
  cudaError_t ret;

	if((width_ * width_factor_) % (process_unit_len_ * block_width_)
			| height_ % block_height_) {
		AERROR << "Invalid block size";
		return false;
	}

  plane_size_ = width_ * height_ * sizeof(unsigned char);
  ret = cudaMalloc(&d_src_, plane_size_ * 2);
  if (ret != cudaSuccess)
    return false;
  ret = cudaStreamCreate(&stream_);
	if(ret != cudaSuccess)
		return false;

	block_size_ = dim3(block_width_, block_height_);
	grid_size_ = dim3((width_ * width_factor_) /
		(process_unit_len_ * block_width_),
		height_ / block_height_);
	
	return true;
}

__host__ bool CudaConvertHandler::Destory() {
	cudaError_t ret;
  ret = cudaStreamDestroy(stream_);
  if(ret != cudaSuccess)
		return false;
  ret = cudaFree(d_src_);
	if(ret != cudaSuccess)
		return false;
	return true;
}

__host__ void CudaConvertHandler::Process(
			unsigned char* src, unsigned char* dst, bool zero_copy) {
	unsigned int flags;
	bool dst_is_mapped = 
		(cudaHostGetFlags(&flags, dst) == cudaSuccess) &&
		(flags & cudaHostAllocMapped);
	
	if(!dst_is_mapped) {
		AERROR << "For performance reasons, only unified memory are allowed";
	}

	if (!zero_copy) {
		cudaMemcpy(d_src_, src, plane_size_ * 2, cudaMemcpyHostToDevice);
	}

  // cudaStreamAttachMemAsync(stream_, dst, 0, cudaMemAttachGlobal);
  // cudaStreamSynchronize(stream_);

	if (!zero_copy) {
		kernel<<<grid_size_, block_size_, 0, stream_>>>(
    		d_src_, dst, width_, width_factor_);
	} else {
		kernel<<<grid_size_, block_size_, 0, stream_>>>(
    		src, dst, width_, width_factor_);	
	}
	

  // cudaStreamAttachMemAsync(stream_, dst, 0, cudaMemAttachHost);
	cudaStreamSynchronize(stream_);

}

}  // namespace camera
}  // namespace drivers
}  // namespace apollo