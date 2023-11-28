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
#include "modules/perception/lidar_detection/detector/cnn_segmentation/feature_generator.h"

#include "modules/perception/common/base/common.h"

namespace apollo {
namespace perception {
namespace lidar {

#define CUDA_KERNEL_LOOP(i, n)                                 \
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < (n); \
       i += blockDim.x * gridDim.x)

#if __CUDA_ARCH__ < 600
__device__ double atomicAdd(double* address, double val) {
  unsigned long long int* address_as_ull = (unsigned long long int*)address;
  unsigned long long int old = *address_as_ull, assumed;
  do {
    assumed = old;
    old = atomicCAS(address_as_ull, assumed,
                    __double_as_longlong(val + __longlong_as_double(assumed)));
    // Note: uses integer comparison to avoid hang in case of NaN
    // (since NaN != NaN)
  } while (assumed != old);
  return __longlong_as_double(old);
}
#endif

__device__ float atomicAdd(float* address, float val) {
  int* address_as_ull = reinterpret_cast<int*>(address);
  int old = *address_as_ull, assumed;
  do {
    assumed = old;
    old = atomicCAS(address_as_ull, assumed,
                    __float_as_int(val + __int_as_float(assumed)));
    // Note: uses integer comparison to avoid hang in case of NaN
    // (since NaN != NaN)
  } while (assumed != old);
  return __int_as_float(old);
}

__device__ double atomic_exch(double* addr, double val) {
  unsigned long long int* m_addr = (unsigned long long int*)addr;
  unsigned long long int old_val = 0;
  old_val = atomicExch(m_addr, __double_as_longlong(val));
  return __longlong_as_double(old_val);
}

__device__ float atomic_exch(float* addr, float val) {
  return atomicExch(addr, (val));
}

// __device__ void atomicMax(double* max_height_addr, double pz) {
//   double old_pz = *max_height_addr;
//   do {
//     old_pz = atomic_exch(max_height_addr, (pz));
//     if (pz < old_pz) {
//       pz = old_pz;
//     }
//   } while (pz > (*max_height_addr));
// }

__device__ void atomicMax(float* max_height_addr, float pz) {
  float old_pz = *max_height_addr;
  do {
    old_pz = atomic_exch(max_height_addr, (pz));
    if (pz < old_pz) {
      pz = old_pz;
    }
  } while (pz > (*max_height_addr));
}

template <typename Dtype>
__global__ void MapKernel(const int n, const base::PointF* pc,
                          Dtype* max_height_data, Dtype* mean_height_data,
                          Dtype* mean_intensity_data, Dtype* count_data,
                          int* point2grid) {
  CUDA_KERNEL_LOOP(i, n) {
    int idx = point2grid[i];
    if (idx == -1) {
      continue;
    }
    Dtype pz = pc[i].z;
    Dtype pi = pc[i].intensity / 255.0;
    atomicMax(&max_height_data[idx], pz);
    atomicAdd(&mean_height_data[idx], pz);
    if (mean_intensity_data != nullptr) {
      atomicAdd(&mean_intensity_data[idx], pi);
    }
    atomicAdd(&count_data[idx], (Dtype)1);
  }
}

template <typename Dtype>
__global__ void AverageKernel(const int n, Dtype* count_data,
                              Dtype* max_height_data, Dtype* mean_height_data,
                              Dtype* mean_intensity_data, Dtype* nonempty_data,
                              Dtype* log_table, const int max_log_num) {
  CUDA_KERNEL_LOOP(i, n) {
    if (count_data[i] < 1e-6) {
      max_height_data[i] = 0;
    } else {
      mean_height_data[i] /= count_data[i];
      if (mean_intensity_data != nullptr) {
        mean_intensity_data[i] /= count_data[i];
      }
      nonempty_data[i] = Dtype(1.0);
    }
    int count = static_cast<int>(count_data[i]);
    if (count < max_log_num) {
      count_data[i] = log_table[count];
    } else {
      count_data[i] = log(1.0 + count);
    }
  }
}

template <typename Dtype>
__global__ void TopIntensityKernel(const int n, Dtype* top_intensity,
                                   base::PointF* pc, Dtype* max_height_data,
                                   int* point2grid) {
  if (top_intensity == nullptr) {
    return;
  }
  CUDA_KERNEL_LOOP(i, n) {
    int idx = point2grid[i];
    if (idx == -1) {
      continue;
    }
    Dtype pz = pc[i].z;
    Dtype pi = pc[i].intensity / 255.0;
    if (pz == max_height_data[idx]) {
      top_intensity[idx] = pi;
    }
  }
}

template <typename Dtype>
__global__ void SetKernel(const int n, const Dtype alpha, Dtype* y) {
  CUDA_KERNEL_LOOP(i, n) { y[i] = alpha; }
}

void FeatureGenerator::GenerateGPU(const base::PointFCloudPtr& pc_ptr,
                                   const std::vector<int>& point2grid) {
  // fill initial value for feature blob
  int map_size = width_ * height_;
  int block_size = (map_size + kGPUThreadSize - 1) / kGPUThreadSize;
  SetKernel<float>
      <<<block_size, kGPUThreadSize>>>(map_size, -5.f, max_height_data_);
  BASE_GPU_CHECK(cudaMemset(mean_height_data_, 0.f, sizeof(float) * map_size));
  BASE_GPU_CHECK(cudaMemset(count_data_, 0.f, sizeof(float) * map_size));
  BASE_GPU_CHECK(cudaMemset(nonempty_data_, 0.f, sizeof(float) * map_size));
  if (use_intensity_feature_) {
    BASE_GPU_CHECK(
        cudaMemset(top_intensity_data_, 0.f, sizeof(float) * map_size));
    BASE_GPU_CHECK(
        cudaMemset(mean_intensity_data_, 0.f, sizeof(float) * map_size));
  }

  // copy cloud data and point2grid from CPU to GPU memory
  size_t cloud_size = pc_ptr->size();
  if (cloud_size > pc_gpu_size_) {
    // cloud data
    BASE_GPU_CHECK(cudaFree(pc_gpu_));
    BASE_GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&pc_gpu_),
                               cloud_size * sizeof(base::PointF)));
    pc_gpu_size_ = cloud_size;
    // point2grid
    BASE_GPU_CHECK(cudaFree(point2grid_gpu_));
    BASE_GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&point2grid_gpu_),
                               cloud_size * sizeof(int)));
  }
  BASE_GPU_CHECK(cudaMemcpy(pc_gpu_, &(pc_ptr->front()),
                             sizeof(base::PointF) * cloud_size,
                             cudaMemcpyHostToDevice));
  BASE_GPU_CHECK(cudaMemcpy(point2grid_gpu_, point2grid.data(),
                             sizeof(int) * cloud_size, cudaMemcpyHostToDevice));

  // compute features
  // float inv_res_x = 0.5 * width_ / range_;
  // float inv_res_y = 0.5 * height_ / range_;
  {
    int block_size = (cloud_size + kGPUThreadSize - 1) / kGPUThreadSize;
    MapKernel<float><<<block_size, kGPUThreadSize>>>(
        cloud_size, pc_gpu_, max_height_data_, mean_height_data_,
        mean_intensity_data_, count_data_, point2grid_gpu_);
    TopIntensityKernel<float><<<block_size, kGPUThreadSize>>>(
        cloud_size, top_intensity_data_, pc_gpu_, max_height_data_,
        point2grid_gpu_);
  }
  {
    int block_size = (map_size + kGPUThreadSize - 1) / kGPUThreadSize;
    float* log_table = log_blob_->mutable_gpu_data() + log_blob_->offset(0, 0);
    AverageKernel<float><<<block_size, kGPUThreadSize>>>(
        map_size, count_data_, max_height_data_, mean_height_data_,
        mean_intensity_data_, nonempty_data_, log_table, kMaxLogNum);
  }
}

void FeatureGenerator::ReleaseGPUMemory() {
  if (pc_gpu_ != nullptr) {
    BASE_GPU_CHECK(cudaFree(pc_gpu_));
  }
  if (point2grid_gpu_ != nullptr) {
    BASE_GPU_CHECK(cudaFree(point2grid_gpu_));
  }
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
