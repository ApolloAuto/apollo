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

#include <algorithm>

#include "cuda_runtime.h"

#include "modules/perception/perception_plugin/lidar_detection/gpu_down_sample/voxel_downsample.h"

namespace apollo {
namespace perception {
namespace lidar {

__global__ void kernel_initializeIndByKey(hashElement *d_hashTable, int number_of_points) {
    int ind = blockIdx.x * blockDim.x + threadIdx.x;
    if (ind < number_of_points) {
        d_hashTable[ind].index_of_point = ind;
        d_hashTable[ind].index_of_voxel = 0;
        d_hashTable[ind].index_of_valid_voxel = 0;
    }
}

__global__ void kernel_getIndexOfVoxelForPoints(
        base::PointF *d_point_cloud,
        hashElement *d_hashTable,
        int number_of_points,
        gridParameters rgd_params) {
    int ind = blockIdx.x * blockDim.x + threadIdx.x;
    if (ind < number_of_points) {
        int ix = (d_point_cloud[ind].x - rgd_params.pointcloud_min_X) / rgd_params.resolution_X;
        int iy = (d_point_cloud[ind].y - rgd_params.pointcloud_min_Y) / rgd_params.resolution_Y;
        int iz = (d_point_cloud[ind].z - rgd_params.pointcloud_min_Z) / rgd_params.resolution_Z;
        d_hashTable[ind].index_of_voxel = ix * rgd_params.number_of_voxel_Y * rgd_params.number_of_voxel_Z
                + iy * rgd_params.number_of_voxel_Z + iz;
    }
}

__global__ void kernel_copyKeys(hashElement *d_hashTable_in, hashElement *d_hashTable_out, int number_of_points) {
    int ind = blockIdx.x * blockDim.x + threadIdx.x;
    if (ind < number_of_points) {
        d_hashTable_out[ind] = d_hashTable_in[ind];
    }
}

__global__ void kernel_setAllPointsToRemove(bool *d_markers, int number_of_points) {
    int ind = blockIdx.x * blockDim.x + threadIdx.x;
    if (ind < number_of_points) {
        d_markers[ind] = false;
    }
}

__global__ void kernel_initializeVoxel(voxel *d_voxel, int number_of_points) {
    int ind = blockIdx.x * blockDim.x + threadIdx.x;
    if (number_of_points > 0 && ind < number_of_points) {
        d_voxel[ind].x = 0;
        d_voxel[ind].y = 0;
        d_voxel[ind].z = 0;
        d_voxel[ind].intensity = 0;
        d_voxel[ind].number_of_points = 0;
    }
}

__global__ void kernel_markFirstPointInVoxel(
        bool *d_markers,
        hashElement *d_hashTable,
        voxel *d_voxel,
        gridParameters rgd_params,
        int number_of_points) {
    int ind = blockIdx.x * blockDim.x + threadIdx.x;
    if (number_of_points > 0 && ind < number_of_points) {
        if (ind == 0) {
            int index_voxel = d_hashTable[ind].index_of_voxel;
            int index_voxel_1 = d_hashTable[ind + 1].index_of_voxel;

            if (index_voxel != index_voxel_1) {
                d_markers[d_hashTable[ind].index_of_point] = true;
            }
        } else if (ind == number_of_points - 1) {
            d_markers[d_hashTable[ind].index_of_point] = true;
        } else {
            int index_voxel = d_hashTable[ind].index_of_voxel;
            int index_voxel_1 = d_hashTable[ind + 1].index_of_voxel;

            if (index_voxel != index_voxel_1) {
                d_markers[d_hashTable[ind].index_of_point] = true;
            }
        }
    }
}

__global__ void kernel_SumPointInVoxel(
        base::PointF *d_point_cloud,
        hashElement *d_hashTable,
        voxel *d_voxel,
        int number_of_points) {
    int ind = blockIdx.x * blockDim.x + threadIdx.x;
    if (number_of_points > 0 && ind < number_of_points) {
        int point_ind = d_hashTable[ind].index_of_point;
        atomicAdd(&d_voxel[d_hashTable[ind].index_of_valid_voxel].x, d_point_cloud[point_ind].x);
        atomicAdd(&d_voxel[d_hashTable[ind].index_of_valid_voxel].y, d_point_cloud[point_ind].y);
        atomicAdd(&d_voxel[d_hashTable[ind].index_of_valid_voxel].z, d_point_cloud[point_ind].z);
        atomicAdd(&d_voxel[d_hashTable[ind].index_of_valid_voxel].intensity, d_point_cloud[point_ind].intensity);
        atomicAdd(&d_voxel[d_hashTable[ind].index_of_valid_voxel].number_of_points, 1);
    }
}

__global__ void kernel_CalCentroidPointInVoxel(voxel *d_voxel, base::PointF *d_point_cloud, int *count) {
    int ind = blockIdx.x * blockDim.x + threadIdx.x;
    if (count[0] > 0 && ind < count[0]) {
        d_point_cloud[ind].x = d_voxel[ind].x / d_voxel[ind].number_of_points;
        d_point_cloud[ind].y = d_voxel[ind].y / d_voxel[ind].number_of_points;
        d_point_cloud[ind].z = d_voxel[ind].z / d_voxel[ind].number_of_points;
        d_point_cloud[ind].intensity = d_voxel[ind].intensity / d_voxel[ind].number_of_points;
    }
}

__global__ void kernel_UpdatePointNumber(int *count, int valid_voxel_number) {
    count[0] = valid_voxel_number + 1;
}

__global__ void kernel_cudaWarmUpGPU() {
    int ind = blockIdx.x * blockDim.x + threadIdx.x;
    ind = ind + 1;
}

cudaError_t VoxelDownSampleCuda::cudaCalculateGridParams(
        base::PointF *d_point_cloud,
        int number_of_points,
        float resolution_X,
        float resolution_Y,
        float resolution_Z,
        gridParameters &out_rgd_params) {
    cudaError_t err = cudaGetLastError();
    try {
        thrust::device_ptr<base::PointF> t_cloud(d_point_cloud);
        err = cudaGetLastError();
        if (err != ::cudaSuccess)
            return err;

        thrust::pair<thrust::device_ptr<base::PointF>, thrust::device_ptr<base::PointF>> minmaxX
                = thrust::minmax_element(t_cloud, t_cloud + number_of_points, compareX());
        err = cudaGetLastError();
        if (err != ::cudaSuccess)
            return err;

        thrust::pair<thrust::device_ptr<base::PointF>, thrust::device_ptr<base::PointF>> minmaxY
                = thrust::minmax_element(t_cloud, t_cloud + number_of_points, compareY());
        err = cudaGetLastError();
        if (err != ::cudaSuccess)
            return err;

        thrust::pair<thrust::device_ptr<base::PointF>, thrust::device_ptr<base::PointF>> minmaxZ
                = thrust::minmax_element(t_cloud, t_cloud + number_of_points, compareZ());
        err = cudaGetLastError();
        if (err != ::cudaSuccess)
            return err;

        base::PointF minX, maxX, minZ, maxZ, minY, maxY;

        err = cudaMemcpy(&minX, minmaxX.first.get(), sizeof(base::PointF), cudaMemcpyDeviceToHost);
        if (err != ::cudaSuccess)
            return err;
        err = cudaMemcpy(&maxX, minmaxX.second.get(), sizeof(base::PointF), cudaMemcpyDeviceToHost);
        if (err != ::cudaSuccess)
            return err;
        err = cudaMemcpy(&minY, minmaxY.first.get(), sizeof(base::PointF), cudaMemcpyDeviceToHost);
        if (err != ::cudaSuccess)
            return err;
        err = cudaMemcpy(&maxY, minmaxY.second.get(), sizeof(base::PointF), cudaMemcpyDeviceToHost);
        if (err != ::cudaSuccess)
            return err;
        err = cudaMemcpy(&minZ, minmaxZ.first.get(), sizeof(base::PointF), cudaMemcpyDeviceToHost);
        if (err != ::cudaSuccess)
            return err;
        err = cudaMemcpy(&maxZ, minmaxZ.second.get(), sizeof(base::PointF), cudaMemcpyDeviceToHost);
        if (err != ::cudaSuccess)
            return err;

        int number_of_voxel_X = ((maxX.x - minX.x) / resolution_X) + 1;
        int number_of_voxel_Y = ((maxY.y - minY.y) / resolution_Y) + 1;
        int number_of_voxel_Z = ((maxZ.z - minZ.z) / resolution_Z) + 1;

        out_rgd_params.number_of_voxel_X = number_of_voxel_X;
        out_rgd_params.number_of_voxel_Y = number_of_voxel_Y;
        out_rgd_params.number_of_voxel_Z = number_of_voxel_Z;
        out_rgd_params.number_of_voxel = number_of_voxel_X * number_of_voxel_Y * number_of_voxel_Z;

        out_rgd_params.pointcloud_max_X = maxX.x;
        out_rgd_params.pointcloud_min_X = minX.x;
        out_rgd_params.pointcloud_max_Y = maxY.y;
        out_rgd_params.pointcloud_min_Y = minY.y;
        out_rgd_params.pointcloud_max_Z = maxZ.z;
        out_rgd_params.pointcloud_min_Z = minZ.z;

        out_rgd_params.resolution_X = resolution_X;
        out_rgd_params.resolution_Y = resolution_Y;
        out_rgd_params.resolution_Z = resolution_Z;
    } catch (thrust::system_error &e) {
        err = cudaGetLastError();
        cudaDeviceReset();
        return err;
    } catch (std::bad_alloc &e) {
        err = cudaGetLastError();
        cudaDeviceReset();
        return err;
    }
    return cudaGetLastError();
}

cudaError_t VoxelDownSampleCuda::cudaCalculateGrid(
        int threads,
        base::PointF *d_point_cloud,
        hashElement *d_hashTable,
        int number_of_points,
        gridParameters rgd_params,
        int *count) {
    cudaError_t err = cudaGetLastError();
    hashElement *d_temp_hashTable;
    cudaMalloc((void **)&d_temp_hashTable, number_of_points * sizeof(hashElement));

    int blocks = number_of_points / threads + 1;
    kernel_initializeIndByKey<<<blocks, threads>>>(d_temp_hashTable, number_of_points);
    err = cudaDeviceSynchronize();
    if (err != ::cudaSuccess)
        return err;

    kernel_getIndexOfVoxelForPoints<<<blocks, threads>>>(d_point_cloud, d_temp_hashTable, number_of_points, rgd_params);
    err = cudaDeviceSynchronize();
    if (err != ::cudaSuccess)
        return err;

    try {
        thrust::device_ptr<hashElement> t_d_temp_hashTable(d_temp_hashTable);
        thrust::sort(t_d_temp_hashTable, t_d_temp_hashTable + number_of_points, compareHashElements());
    } catch (thrust::system_error &e) {
        err = cudaGetLastError();
        return err;
    } catch (std::bad_alloc &e) {
        err = cudaGetLastError();
        return err;
    }

    kernel_copyKeys<<<blocks, threads>>>(d_temp_hashTable, d_hashTable, number_of_points);
    err = cudaDeviceSynchronize();
    if (err != ::cudaSuccess)
        return err;

    hashElement *h_hashTable = (hashElement *)malloc(number_of_points * sizeof(hashElement));
    cudaMemcpy(h_hashTable, d_hashTable, number_of_points * sizeof(hashElement), cudaMemcpyDeviceToHost);

    int valid_voxel_number = 0;
    for (size_t i = 0; i < number_of_points; ++i) {
        if (i > 0 && (h_hashTable[i].index_of_voxel != h_hashTable[i - 1].index_of_voxel)) {
            valid_voxel_number += 1;
        }
        h_hashTable[i].index_of_valid_voxel = valid_voxel_number;
    }
    cudaMemcpy(d_hashTable, h_hashTable, number_of_points * sizeof(hashElement), cudaMemcpyHostToDevice);

    free(h_hashTable);

    kernel_UpdatePointNumber<<<1, 1>>>(count, valid_voxel_number);
    err = cudaDeviceSynchronize();
    if (err != ::cudaSuccess)
        return err;

    err = cudaFree(d_temp_hashTable);
    return err;
}

cudaError_t VoxelDownSampleCuda::cudaDownSample(
        int threads,
        base::PointF *d_point_cloud,
        bool *d_markers,
        hashElement *d_hashTable,
        voxel *d_voxel,
        int number_of_points,
        gridParameters rgd_params,
        bool use_centroid_downsample,
        int *count) {
    int blocks = number_of_points / threads + 1;
    cudaError_t err = cudaGetLastError();

    if (use_centroid_downsample) {
        kernel_initializeVoxel<<<blocks, threads>>>(d_voxel, number_of_points);
        err = cudaDeviceSynchronize();
        if (err != ::cudaSuccess)
            return err;

        kernel_SumPointInVoxel<<<blocks, threads>>>(d_point_cloud, d_hashTable, d_voxel, number_of_points);
        err = cudaDeviceSynchronize();
        if (err != ::cudaSuccess)
            return err;

        kernel_CalCentroidPointInVoxel<<<blocks, threads>>>(d_voxel, d_point_cloud, count);
        err = cudaDeviceSynchronize();
    } else {
        kernel_setAllPointsToRemove<<<blocks, threads>>>(d_markers, number_of_points);
        err = cudaDeviceSynchronize();
        if (err != ::cudaSuccess)
            return err;

        // Select first point in voxel as the final output point
        kernel_markFirstPointInVoxel<<<blocks, threads>>>(
                d_markers, d_hashTable, d_voxel, rgd_params, number_of_points);
        err = cudaDeviceSynchronize();
    }
    return err;
}

cudaError_t VoxelDownSampleCuda::cudaWarmUpGPU() {
    kernel_cudaWarmUpGPU<<<1, 1>>>();
    // cudaDeviceSynchronize makes the host (The CPU) wait
    // until the device (The GPU) have finished executing ALL the threads you have
    // started
    cudaDeviceSynchronize();
    return cudaGetLastError();
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
