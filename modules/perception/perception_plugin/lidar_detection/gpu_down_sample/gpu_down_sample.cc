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

#include "modules/perception/perception_plugin/lidar_detection/gpu_down_sample/gpu_down_sample.h"

#include <algorithm>

#include "modules/perception/common/lidar/common/pcl_util.h"
#include "modules/perception/common/util.h"

namespace apollo {
namespace perception {
namespace lidar {

bool GpuDownSample::Init(const DownSampleInitOptions& options) {
    std::string config_file = GetConfigFile(options.config_path, options.config_file);
    GpuDownSampleConfig config;
    ACHECK(cyber::common::GetProtoFromFile(config_file, &config));
    downsample_voxel_size_x_ = config.downsample_voxel_size_x();
    downsample_voxel_size_y_ = config.downsample_voxel_size_y();
    downsample_voxel_size_z_ = config.downsample_voxel_size_z();
    downsample_use_centroid = config.downsample_use_centroid();

    AINFO << "GpuDownSample plugin init success.";
    return true;
}

bool GpuDownSample::Process(const DownSampleOptions& options, base::PointFCloudPtr& cloud_ptr) {
    base::PointF* h_point_cloud;
    base::PointF* d_point_cloud;
    cudaError_t err = ::cudaSuccess;
    unsigned int flags = cudaHostAllocMapped;
    err = cudaHostAlloc(&h_point_cloud, cloud_ptr->size() * sizeof(base::PointF), flags);
    if (err != ::cudaSuccess) {
        AERROR << "cudaHostAlloc returns failed";
    }
    err = cudaHostGetDevicePointer(&d_point_cloud, h_point_cloud, 0);
    if (err != ::cudaSuccess) {
        AERROR << "cudaHostGetDevicePointer returns failed";
    }
    for (int i = 0; i < cloud_ptr->size(); i++) {
        h_point_cloud[i].x = cloud_ptr->at(i).x;
        h_point_cloud[i].y = cloud_ptr->at(i).y;
        h_point_cloud[i].z = cloud_ptr->at(i).z;
        h_point_cloud[i].intensity = cloud_ptr->at(i).intensity;
    }

    // DownSample
    int filtered_point_count = cloud_ptr->size();
    if (!downSample(
                d_point_cloud,
                filtered_point_count,
                downsample_voxel_size_x_,
                downsample_voxel_size_y_,
                downsample_voxel_size_z_,
                downsample_use_centroid)) {
        AERROR << "downSample is wrong!";
    }
    cudaDeviceSynchronize();

    // After DownSample
    cloud_ptr->resize(filtered_point_count);
    for (int i = 0; i < filtered_point_count; i++) {
        cloud_ptr->at(i).x = h_point_cloud[i].x;
        cloud_ptr->at(i).y = h_point_cloud[i].y;
        cloud_ptr->at(i).z = h_point_cloud[i].z;
        cloud_ptr->at(i).intensity = h_point_cloud[i].intensity;
    }
    cudaFreeHost(h_point_cloud);
    return true;
}

void GpuDownSample::warmUpGPU() {
    cudaError_t err = ::cudaSuccess;
    err = cudaSetDevice(0);
    if (err != ::cudaSuccess) {
        return;
    }

    err = voxel_downSample_ptr->cudaWarmUpGPU();
    if (err != ::cudaSuccess) {
        return;
    }
}

int GpuDownSample::getNumberOfAvailableThreads() {
    cudaDeviceProp prop;
    cudaGetDeviceProperties(&prop, 0);

    int threads = 0;
    // major, minor are the major and minor revision numbers
    // defining the device's compute capability
    if (prop.major == 2) {
        threads = prop.maxThreadsPerBlock / 2;
    } else if (prop.major > 2) {
        threads = prop.maxThreadsPerBlock;
    } else {
        return 0;
    }

    return threads;
}

bool GpuDownSample::downSample(
        base::PointF* point_cloud,
        int& point_cloud_size,
        float resolution_x,
        float resolution_y,
        float resolution_z,
        bool use_centroid_downsample) {
    cudaError_t err = ::cudaSuccess;
    err = cudaSetDevice(0);
    if (err != ::cudaSuccess) {
        return false;
    }
    gridParameters rgd_params;
    hashElement* d_hashTable = NULL;
    voxel* d_voxel = NULL;
    bool* d_markers;
    bool* h_markers;
    int* count;
    int threads = getNumberOfAvailableThreads();
    if (threads == 0) {
        return false;
    }

    err = voxel_downSample_ptr->cudaCalculateGridParams(
            point_cloud, point_cloud_size, resolution_x, resolution_y, resolution_z, rgd_params);
    if (err != ::cudaSuccess) {
        return false;
    }

    err = cudaMalloc(reinterpret_cast<void**>(&d_hashTable), point_cloud_size * sizeof(hashElement));
    if (err != ::cudaSuccess) {
        return false;
    }
    err = cudaMalloc(reinterpret_cast<void**>(&count), sizeof(int));
    if (err != ::cudaSuccess) {
        return false;
    }
    if (use_centroid_downsample) {
        err = cudaMalloc(reinterpret_cast<void**>(&d_voxel), point_cloud_size * sizeof(voxel));
        if (err != ::cudaSuccess) {
            return false;
        }
    }
    err = voxel_downSample_ptr->cudaCalculateGrid(
            threads, point_cloud, d_hashTable, point_cloud_size, rgd_params, count);
    if (err != ::cudaSuccess) {
        return false;
    }

    if (!use_centroid_downsample) {
        err = cudaMalloc(reinterpret_cast<void**>(&d_markers), point_cloud_size * sizeof(bool));
        if (err != ::cudaSuccess) {
            return false;
        }
    }
    err = voxel_downSample_ptr->cudaDownSample(
            threads,
            point_cloud,
            d_markers,
            d_hashTable,
            d_voxel,
            point_cloud_size,
            rgd_params,
            use_centroid_downsample,
            count);
    if (err != ::cudaSuccess) {
        return false;
    }

    if (use_centroid_downsample) {
        int* host_count = new int[1];
        err = cudaMemcpy(host_count, count, sizeof(int), cudaMemcpyDeviceToHost);
        if (err != ::cudaSuccess) {
            return false;
        }
        point_cloud_size = host_count[0];
    } else {
        h_markers = reinterpret_cast<bool*>(malloc(point_cloud_size * sizeof(bool)));
        err = cudaMemcpy(h_markers, d_markers, point_cloud_size * sizeof(bool), cudaMemcpyDeviceToHost);
        if (err != ::cudaSuccess) {
            return false;
        }
        int valid_size = 0;
        for (size_t i = 0; i < point_cloud_size; i++) {
            if (h_markers[i]) {
                point_cloud[valid_size] = point_cloud[i];
                valid_size += 1;
            }
        }
        point_cloud_size = valid_size;
    }

    if (!use_centroid_downsample) {
        free(h_markers);
    }

    err = cudaFree(d_hashTable);
    d_hashTable = NULL;
    if (err != ::cudaSuccess) {
        return false;
    }
    err = cudaFree(count);
    count = NULL;
    if (err != ::cudaSuccess) {
        return false;
    }
    if (use_centroid_downsample) {
        err = cudaFree(d_voxel);
        d_voxel = NULL;
        if (err != ::cudaSuccess) {
            return false;
        }
    } else {
        err = cudaFree(d_markers);
        d_markers = NULL;
        if (err != ::cudaSuccess) {
            return false;
        }
    }

    return true;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
