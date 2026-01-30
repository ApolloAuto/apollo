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

#include <stdio.h>

#include "modules/perception/common/base/blob.h"
#include "modules/perception/common/base/common.h"
#include "modules/perception/pointcloud_semantics/parser/pointwise_unet/pointwise_unet.h"

namespace apollo {
namespace perception {
namespace lidar {

#define CUDA_KERNEL_LOOP(i, n) for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < (n); i += blockDim.x * gridDim.x)

__device__ float atomicAdd(float* address, float val) {
    int* address_as_ull = reinterpret_cast<int*>(address);
    int old = *address_as_ull, assumed;
    do {
        assumed = old;
        old = atomicCAS(address_as_ull, assumed, __float_as_int(val + __int_as_float(assumed)));
        // Note: uses integer comparison to avoid hang in case of NaN
        // (since NaN != NaN)
    } while (assumed != old);
    return __int_as_float(old);
}

__device__ float atomic_exch(float* addr, float val) {
    return atomicExch(addr, (val));
}

__device__ void atomicMax(float* max_height_addr, float pz) {
    float old_pz = *max_height_addr;
    do {
        old_pz = atomic_exch(max_height_addr, (pz));
        if (pz < old_pz) {
            pz = old_pz;
        }
    } while (pz > (*max_height_addr));
}

__global__ void ComputePointCloudSumKernel(
        const base::PointF* pc_gpu,
        const int* point_index,
        const int* grid_index,
        int n,
        int unet_length,
        float* xyz_mean) {
    CUDA_KERNEL_LOOP(i, n) {
        int pidx = point_index[i];
        base::PointF pt = pc_gpu[pidx];
        // compute sum x,y,z and point-number
        int pillar_idx = grid_index[i * 2] * unet_length + grid_index[i * 2 + 1];
        atomicAdd(&xyz_mean[pillar_idx * 4 + 0], pt.x);
        atomicAdd(&xyz_mean[pillar_idx * 4 + 1], pt.y);
        atomicAdd(&xyz_mean[pillar_idx * 4 + 2], pt.z);
        atomicAdd(&xyz_mean[pillar_idx * 4 + 3], 1.0);
    }
}

__global__ void VoxelsAncCoorsFeaturesKernel(
        const base::PointF* pc_gpu,
        const int* point_index,
        const int* grid_index,
        const float* xyz_mean,
        int n,
        int unet_length,
        float resolution,
        float min_x,
        float min_y,
        float* voxels,
        float* coors) {
    CUDA_KERNEL_LOOP(i, n) {
        int pidx = point_index[i];
        base::PointF pt = pc_gpu[pidx];
        int pillar_idx = (grid_index[i * 2] * unet_length + grid_index[i * 2 + 1]) * 4;
        voxels[i * 10 + 0] = pt.x;
        voxels[i * 10 + 1] = pt.y;
        voxels[i * 10 + 2] = pt.z;
        voxels[i * 10 + 3] = pt.intensity;
        voxels[i * 10 + 4] = pt.x - xyz_mean[pillar_idx + 0] / (xyz_mean[pillar_idx + 3] + 1e-6);
        voxels[i * 10 + 5] = pt.y - xyz_mean[pillar_idx + 1] / (xyz_mean[pillar_idx + 3] + 1e-6);
        voxels[i * 10 + 6] = pt.z - xyz_mean[pillar_idx + 2] / (xyz_mean[pillar_idx + 3] + 1e-6);
        voxels[i * 10 + 7] = pt.x - (static_cast<float>(grid_index[i * 2 + 1]) * resolution + 0.5 * resolution + min_x);
        voxels[i * 10 + 8] = pt.y - (static_cast<float>(grid_index[i * 2]) * resolution + 0.5 * resolution + min_y);
        voxels[i * 10 + 9] = pt.z - 0.0;
        // get coors
        coors[i * 4 + 0] = 0;
        coors[i * 4 + 1] = 0;
        coors[i * 4 + 2] = grid_index[i * 2];
        coors[i * 4 + 3] = grid_index[i * 2 + 1];
    }
}

__global__ void BevFeatureKernel(
        const int* grid_index,
        const float* point_feats,
        int n,
        int unet_length,
        int unet_width,
        float* bev_feature) {
    CUDA_KERNEL_LOOP(i, n) {
        for (int j = 0; j < 64; j++) {
            int index = j * unet_length * unet_width + grid_index[i * 2] * unet_length + grid_index[i * 2 + 1];
            atomicMax(&bev_feature[index], point_feats[i * 64 + j]);
        }
    }
}

void PointWiseUnet::GeneratePfnFeatureGPU() {
    int cloud_size = original_cloud_->size();
    // get valid pointcloud index and (grid_y, grid_x)
    GetValidPointcloud();
    int valid_size = valid_point_index_.size();
    // cuda-malloc point-cloud data
    if (pc_gpu_ != nullptr) {
        BASE_GPU_CHECK(cudaFree(pc_gpu_));  // free cuda memory
    }
    BASE_GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&pc_gpu_), cloud_size * sizeof(base::PointF)));
    // memory copy from original_cloud_ to pc_gpu_
    BASE_GPU_CHECK(cudaMemcpy(
            pc_gpu_, &(original_cloud_->front()), sizeof(base::PointF) * cloud_size, cudaMemcpyHostToDevice));
    // pointcloud index
    int* point_index_gpu = nullptr;
    BASE_GPU_CHECK(cudaMalloc((void**)&point_index_gpu, sizeof(int) * valid_size));
    BASE_GPU_CHECK(cudaMemcpy(
            point_index_gpu,
            valid_point_index_.data(),
            sizeof(int) * valid_point_index_.size(),
            cudaMemcpyHostToDevice));
    // pointcloud grid index
    int* grid_index_gpu = nullptr;
    BASE_GPU_CHECK(cudaMalloc((void**)&grid_index_gpu, sizeof(int) * valid_size * 2));
    BASE_GPU_CHECK(cudaMemcpy(
            grid_index_gpu, grid_index_vec_.data(), sizeof(int) * grid_index_vec_.size(), cudaMemcpyHostToDevice));
    // x,y,z means. shape: (4, pillar_num), 4: [x, y, z, point_number]
    float* xyz_mean_gpu = nullptr;
    BASE_GPU_CHECK(cudaMalloc((void**)&xyz_mean_gpu, sizeof(float) * unet_width_ * unet_length_ * 4));
    // set xyz_mean_gpu to 0.0
    BASE_GPU_CHECK(cudaMemsetAsync(xyz_mean_gpu, 0.0f, sizeof(float) * unet_width_ * unet_length_ * 4, stream_));

    int block_size = (valid_size + kGPUThreadSize - 1) / kGPUThreadSize;

    ComputePointCloudSumKernel<<<block_size, kGPUThreadSize, 0, stream_>>>(
            pc_gpu_, point_index_gpu, grid_index_gpu, valid_size, unet_length_, xyz_mean_gpu);

    // coors feature
    coors_->Reshape({static_cast<int>(valid_point_index_.size()), 4});
    float* coors_gpu = coors_->mutable_gpu_data();

    // voxels feature
    voxels_->Reshape({static_cast<int>(valid_point_index_.size()), 10});
    float* voxel_gpu = voxels_->mutable_gpu_data();

    VoxelsAncCoorsFeaturesKernel<<<block_size, kGPUThreadSize, 0, stream_>>>(
            pc_gpu_,
            point_index_gpu,
            grid_index_gpu,
            xyz_mean_gpu,
            valid_size,
            unet_length_,
            resolution_,
            min_x_,
            min_y_,
            voxel_gpu,
            coors_gpu);

    // output
    pfn_point_feats_->Reshape({static_cast<int>(valid_point_index_.size()), 64});

    // free cuda memory
    BASE_GPU_CHECK(cudaFree(point_index_gpu));
    BASE_GPU_CHECK(cudaFree(grid_index_gpu));
    BASE_GPU_CHECK(cudaFree(xyz_mean_gpu));
}

void PointWiseUnet::GenerateBackboneFeatureGPU() {
    int valid_size = valid_point_index_.size();
    // pointcloud grid index
    int* grid_index_gpu = nullptr;
    BASE_GPU_CHECK(cudaMalloc((void**)&grid_index_gpu, sizeof(int) * grid_index_vec_.size()));
    BASE_GPU_CHECK(cudaMemcpy(
            grid_index_gpu, grid_index_vec_.data(), sizeof(int) * grid_index_vec_.size(), cudaMemcpyHostToDevice));

    // get point_feats
    point_feats_->Reshape(pfn_point_feats_->shape());
    point_feats_->set_gpu_data(pfn_point_feats_->mutable_gpu_data());
    float* point_feats_gpu = pfn_point_feats_->mutable_gpu_data();

    // bev feature
    bev_feature_->Reshape({1, 64, unet_length_, unet_width_});
    float* bev_feature_gpu = bev_feature_->mutable_gpu_data();
    BASE_GPU_CHECK(cudaMemsetAsync(bev_feature_gpu, 0.f, sizeof(float) * unet_width_ * unet_length_ * 64, stream_));

    int block_size = (valid_size + kGPUThreadSize - 1) / kGPUThreadSize;

    // fill bev feature
    BevFeatureKernel<<<block_size, kGPUThreadSize, 0, stream_>>>(
            grid_index_gpu, point_feats_gpu, valid_size, unet_length_, unet_width_, bev_feature_gpu);

    // output
    seg_pred_blob_->Reshape({max_point_number_, 7});

    // free cuda memory
    BASE_GPU_CHECK(cudaFree(grid_index_gpu));
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
