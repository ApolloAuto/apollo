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

#include "modules/map/pnc_map/cuda_util.h"

#include "cyber/common/log.h"

#include <cuda_runtime_api.h>

namespace apollo {
namespace pnc_map {

constexpr std::size_t kDeviceVecSize = 2000;

CudaNearestSegment::CudaNearestSegment() {
  host_seg_ = new CudaLineSegment2d[kDeviceVecSize];
  cudaError_t cudaStatus;
  cudaStatus = cudaMalloc((void**)&dev_dist_, sizeof(double) * kDeviceVecSize);
  CHECK_EQ(cudaStatus, cudaSuccess);
  cudaStatus =
      cudaMalloc((void**)&dev_seg_, sizeof(CudaLineSegment2d) * kDeviceVecSize);
  CHECK_EQ(cudaStatus, cudaSuccess);
  cublasCreate(&handle_);
}

CudaNearestSegment::~CudaNearestSegment() {
  delete[] host_seg_;
  cudaFree(dev_dist_);
  cudaFree(dev_seg_);
}

__device__ double distance_square(const CudaLineSegment2d seg, double x,
                                  double y) {
  double x1x = x - seg.x1;
  double y1y = y - seg.y1;
  double x1x2 = seg.x2 - seg.x1;
  double y1y2 = seg.y2 - seg.y1;
  double dot = x1x * x1x2 + y1y * y1y2;
  if (dot < 0) {
    return x1x * x1x + y1y * y1y;
  } else if (dot > x1x2 * x1x2 + y1y2 * y1y2) {
    double x2x = x - seg.x2;
    double y2y = y - seg.y2;
    return x2x * x2x + y2y * y2y;
  } else {
    double prod = x1x * y1y2 - y1y * x1x2;
    return prod * prod;
  }
}

__host__ bool CudaNearestSegment::UpdateLineSegment(
    const std::vector<apollo::common::math::LineSegment2d>& segments) {
  size_ = std::min(kDeviceVecSize, segments.size());
  for (std::size_t i = 0; i < size_; ++i) {
    host_seg_[i].x1 = segments[i].start().x();
    host_seg_[i].y1 = segments[i].start().y();
    host_seg_[i].x2 = segments[i].end().x();
    host_seg_[i].y2 = segments[i].end().y();
  }
  cudaError_t cudaStatus;
  cudaStatus =
      cudaMemcpy(dev_seg_, host_seg_, size_ * sizeof(CudaLineSegment2d),
                 cudaMemcpyHostToDevice);
  if (cudaStatus != cudaSuccess) {
    AERROR << "Failed to copy to cuda device";
    return false;
  }
  return true;
}

__global__ void DistanceSquare(double x, double y, CudaLineSegment2d* dev_seg,
                               double* dev_dist, int32_t size) {
  int32_t index = blockDim.x * blockIdx.x + threadIdx.x;
  if (index >= size) {
    return;
  }
  const auto& seg = dev_seg[index];
  double x1x = x - seg.x1;
  double y1y = y - seg.y1;
  double x1x2 = seg.x2 - seg.x1;
  double y1y2 = seg.y2 - seg.y1;
  double dot = x1x * x1x2 + y1y * y1y2;
  if (dot < 0) {
    dev_dist[index] = x1x * x1x + y1y * y1y;
  } else if (dot > x1x2 * x1x2 + y1y2 * y1y2) {
    double x2x = x - seg.x2;
    double y2y = y - seg.y2;
    dev_dist[index] = x2x * x2x + y2y * y2y;
  } else {
    double prod = x1x * y1y2 - y1y * x1x2;
    dev_dist[index] = prod * prod;
  }
}

int CudaNearestSegment::FindNearestSegment(double x, double y) {
  DistanceSquare<<<(kDeviceVecSize + 511) / 512, 512>>>(x, y, dev_seg_,
                                                        dev_dist_, size_);
  cublasStatus_t stat;
  int min_index = 0;
  stat = cublasIdamin(handle_, size_, dev_dist_, 1, &min_index);
  if (stat != CUBLAS_STATUS_SUCCESS) {
    AERROR << "Failed to find min element in segments";
    return -1;
  }
  return min_index - 1;
}
}  // namespace pnc_map
}  // namespace apollo
