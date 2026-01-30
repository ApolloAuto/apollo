/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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

#include <iostream>
#include <vector>

#include <thrust/device_ptr.h>
#include <thrust/device_vector.h>
#include <thrust/extrema.h>
#include <thrust/host_vector.h>
#include <thrust/sort.h>

#include "modules/perception/common/base/object_pool_types.h"
#include "modules/perception/common/base/point_cloud_util.h"

namespace apollo {
namespace perception {
namespace lidar {

struct hashElement {
    int index_of_point;
    int index_of_voxel;
    int index_of_valid_voxel;
};

struct voxel {
    float x;
    float y;
    float z;
    float intensity;
    int number_of_points;
};

struct gridParameters {
    float pointcloud_min_X;
    float pointcloud_min_Y;
    float pointcloud_min_Z;
    float pointcloud_max_X;
    float pointcloud_max_Y;
    float pointcloud_max_Z;
    int number_of_voxel_X;
    int number_of_voxel_Y;
    int number_of_voxel_Z;
    int number_of_voxel;
    float resolution_X;
    float resolution_Y;
    float resolution_Z;
};

struct compareHashElements {
    __host__ __device__ bool operator()(hashElement l, hashElement r) {
        return l.index_of_voxel < r.index_of_voxel;
    }
};

struct compareX {
    __host__ __device__ bool operator()(base::PointF lp, base::PointF rp) {
        return lp.x < rp.x;
    }
};

struct compareY {
    __host__ __device__ bool operator()(base::PointF lp, base::PointF rp) {
        return lp.y < rp.y;
    }
};

struct compareZ {
    __host__ __device__ bool operator()(base::PointF lp, base::PointF rp) {
        return lp.z < rp.z;
    }
};

class VoxelDownSampleCuda {
public:
    VoxelDownSampleCuda();
    ~VoxelDownSampleCuda();

    cudaError_t cudaCalculateGridParams(
            base::PointF *d_point_cloud,
            int number_of_points,
            float resolution_X,
            float resolution_Y,
            float resolution_Z,
            gridParameters &out_rgd_params);

    cudaError_t cudaCalculateGrid(
            int threads,
            base::PointF *d_point_cloud,
            hashElement *d_hashTable,
            int number_of_points,
            gridParameters rgd_params,
            int *count);

    cudaError_t cudaDownSample(
            int threads,
            base::PointF *d_point_cloud,
            bool *d_markers,
            hashElement *d_hashTable,
            voxel *d_voxel,
            int number_of_points,
            gridParameters rgd_params,
            bool use_centroid_downsample,
            int *count);

    cudaError_t cudaWarmUpGPU();
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
