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

#include <stdio.h>

#include "modules/perception/lidar_cpdet_detection/detector/cpdet/cpdet.h"
#include "modules/perception/common/lidar/common/lidar_timer.h"

#include "thrust/functional.h"
#include "thrust/sort.h"

namespace apollo {
namespace perception {
namespace lidar {

#define CUDA_KERNEL_LOOP(i, n) \
for (int i = blockIdx.x * blockDim.x + threadIdx.x; \
      i < (n); \
      i += blockDim.x * gridDim.x)

#define DIVUP(m, n) ((m) / (n) + ((m) % (n) > 0))

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

__device__ int atomicAdd(int* address, int val) {
    int* address_as_ull = address;
    int old = *address_as_ull, assumed;
    do {
        assumed = old;
        old = atomicCAS(address_as_ull, assumed, (val +assumed));
        // Note: uses integer comparison to avoid hang in case of NaN
        // (since NaN != NaN)
    } while (assumed != old);
  return old;
}

__device__ double atomic_exch(double* addr, double val) {
  unsigned long long int *m_addr = (unsigned long long int*) addr;
  unsigned long long int old_val = 0;
  old_val = atomicExch(m_addr, __double_as_longlong(val));
  return __longlong_as_double(old_val);
}

__device__ float atomic_exch(float* addr, float val) {
  return atomicExch(addr, (val));
}

__device__ void atomicMax(double* max_height_addr, double pz) {
    double old_pz = *max_height_addr;
    do {
        old_pz = atomic_exch(max_height_addr, (pz));
        if (pz < old_pz) {
            pz = old_pz;
        }
    } while (pz > (*max_height_addr));
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

__host__ __device__
float sigmoid_gpu(float x) {
  return 1.0 / (1.0 + exp(-x));
}

__host__ __device__
float square_root_gpu(float x, float y) {
  return sqrt(x * x + y * y);
}

__host__ __device__
float jaccard_overlap_gpu(const float *bbox1, const float *bbox2) {
    if (bbox2[0] > bbox1[2] || bbox2[2] < bbox1[0] ||
        bbox2[1] > bbox1[3] || bbox2[3] < bbox1[1]) {
        return float(0.); // NOLINT
    } else {
        float left = std::max(bbox1[0], bbox2[0]);
        float right = std::min(bbox1[2], bbox2[2]);
        float top = std::max(bbox1[1], bbox2[1]);
        float bottom = std::min(bbox1[3], bbox2[3]);
        float width = std::max(right - left + 1, 0.f);
        float height = std::max(bottom - top + 1, 0.f);
        float interS = width * height;
        float Sa = (bbox1[2] - bbox1[0] + 1) * (bbox1[3] - bbox1[1] + 1);
        float Sb = (bbox2[2] - bbox2[0] + 1) * (bbox2[3] - bbox2[1] + 1);
        return interS / (Sa + Sb - interS);
    }
}

// rotate_overlap
__host__ __device__
float trangle_area(float* a, float* b, float* c) {
    return ((a[0] - c[0]) * (b[1] - c[1]) - (a[1] - c[1]) * (b[0] - c[0])) / 2.f;
}

__host__ __device__
void sort_vertex_in_convex_polygon(float* int_pts, int num_of_inter) {
    if (num_of_inter == 0) {
        return;
    }
    float center_x = 0.f;
    float center_y = 0.f;
    for (int i = 0; i < num_of_inter; i++) {
        center_x += int_pts[2 * i];
        center_y += int_pts[2 * i + 1];
    }
    center_x /= num_of_inter;
    center_y /= num_of_inter;
    float v0;
    float v1;
    float vs[16];
    for (int i = 0; i < num_of_inter; i++) {
        v0 = int_pts[2 * i] - center_x;
        v1 = int_pts[2 * i + 1] - center_y;
        float d = sqrt(v0 * v0 + v1 * v1);
        v0 = v0 / d;
        v1 = v1 / d;
        if (v1 < 0) {
            v0 = -2 - v0;
        }
        vs[i] = v0;
    }
    int j = 0;
    float temp = 0.f;
    for (int i = 0; i < num_of_inter; i++) {
        if (vs[i - 1] > vs[i]) {
            temp = vs[i];
            float tx = int_pts[2 * i];
            float ty = int_pts[2 * i + 1];
            j = i;
            while (j > 0 && vs[j - 1] > temp) {
                vs[j] = vs[j - 1];
                int_pts[j * 2] = int_pts[j * 2 - 2];
                int_pts[j * 2 + 1] = int_pts[j * 2 - 1];
                j -= 1;
            }
            vs[j] = temp;
            int_pts[j * 2] = tx;
            int_pts[j * 2 + 1] = ty;
        }
    }
}

__host__ __device__
bool line_segment_intersection(const float* pts1, const float* pts2, int i, int j, float* temp_pts) {
    float A0 = pts1[2 * i];
    float A1 = pts1[2 * i + 1];

    float B0 = pts1[2 * ((i + 1) % 4)];
    float B1 = pts1[2 * ((i + 1) % 4) + 1];

    float C0 = pts2[2 * j];
    float C1 = pts2[2 * j + 1];

    float D0 = pts2[2 * ((j + 1) % 4)];
    float D1 = pts2[2 * ((j + 1) % 4) + 1];
    float BA0 = B0 - A0;
    float BA1 = B1 - A1;
    float DA0 = D0 - A0;
    float CA0 = C0 - A0;
    float DA1 = D1 - A1;
    float CA1 = C1 - A1;
    bool acd = DA1 * CA0 > CA1 * DA0;
    bool bcd = (D1 - B1) * (C0 - B0) > (C1 - B1) * (D0 - B0);
    if (acd != bcd) {
        bool abc = CA1 * BA0 > BA1 * CA0;
        bool abd = DA1 * BA0 > BA1 * DA0;
        if (abc != abd) {
            float DC0 = D0 - C0;
            float DC1 = D1 - C1;
            float ABBA = A0 * B1 - B0 * A1;
            float CDDC = C0 * D1 - D0 * C1;
            float DH = BA1 * DC0 - BA0 * DC1;
            float Dx = ABBA * DC0 - BA0 * CDDC;
            float Dy = ABBA * DC1 - BA1 * CDDC;
            temp_pts[0] = Dx / DH;
            temp_pts[1] = Dy / DH;
            return true;
        }
    }
    return false;
}

__host__ __device__
bool point_in_quadrilateral(float pt_x, float pt_y, const float* corners) {
    float ab0 = corners[2] - corners[0];
    float ab1 = corners[3] - corners[1];

    float ad0 = corners[6] - corners[0];
    float ad1 = corners[7] - corners[1];

    float ap0 = pt_x - corners[0];
    float ap1 = pt_y - corners[1];

    float abab = ab0 * ab0 + ab1 * ab1;
    float abap = ab0 * ap0 + ab1 * ap1;
    float adad = ad0 * ad0 + ad1 * ad1;
    float adap = ad0 * ap0 + ad1 * ap1;

    return abab >= abap && abap >= 0 && adad >= adap && adap >= 0;
}

__host__ __device__
float rotate_inter_gpu(const float *corners1, const float *corners2) {
    float intersection_corners[16];
    // quadrilateral_intersection
    int num_of_inter = 0;
    for (int i = 0; i < 4; i++) {
        if (point_in_quadrilateral(corners1[2 * i], corners1[2 * i + 1], corners2)) {
            intersection_corners[num_of_inter * 2] = corners1[2 * i];
            intersection_corners[num_of_inter * 2 + 1] = corners1[2 * i + 1];
            num_of_inter += 1;
        }
        if (point_in_quadrilateral(corners2[2 * i], corners2[2 * i + 1], corners1)) {
            intersection_corners[num_of_inter * 2] = corners2[2 * i];
            intersection_corners[num_of_inter * 2 + 1] = corners2[2 * i + 1];
            num_of_inter += 1;
        }
    }
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            if (line_segment_intersection(corners1, corners2, i, j, intersection_corners + num_of_inter * 2)) {
                ++num_of_inter;
            }
        }
    }

    sort_vertex_in_convex_polygon(intersection_corners, num_of_inter);
    // area
    float area_val = 0.f;
    for (int i = 0; i < num_of_inter - 2; i++) {
        area_val += abs(trangle_area(intersection_corners,
            intersection_corners + 2 * (i + 1),
            intersection_corners + 2 * (i + 2)));
    }
    return area_val;
}

template<typename Dtype>
__global__ void MapKernel(const int n, const base::PointF* pc_gpu,
    const int* point2grid, Dtype* max_height_data, Dtype* mean_height_data,
    Dtype* mean_intensity_data, Dtype* count_data) {
    CUDA_KERNEL_LOOP(i, n) {
        int idx = point2grid[i];
        if (idx == -1) {
            continue;
        }
        Dtype pz = pc_gpu[i].z;
        Dtype pi = pc_gpu[i].intensity / 255.0;
        atomicMax(&max_height_data[idx], pz);
        atomicAdd(&mean_height_data[idx], pz);
        if (mean_intensity_data != nullptr) {
            atomicAdd(&mean_intensity_data[idx], pi);
        }
        atomicAdd(&count_data[idx], (Dtype) 1);
    }
}

template<typename Dtype>
__global__ void AverageKernel(const int n,  Dtype* count_data,
    Dtype* max_height_data, Dtype* mean_height_data,
    Dtype* mean_intensity_data, Dtype* nonempty_data) {
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
        count_data[i] = static_cast<int>(std::log(static_cast<float>(1.0 + count)));
    }
}

template<typename Dtype>
__global__ void TopIntensityKernel(const int n, const base::PointF* pc_gpu, 
    const int* point2grid, Dtype* top_intensity_data, Dtype* max_height_data) {
    CUDA_KERNEL_LOOP(i, n) {
        int idx = point2grid[i];
        if (idx == -1) {
            continue;
        }
        Dtype pz = pc_gpu[i].z;
        Dtype pi = pc_gpu[i].intensity / 255.0;
        // printf("voxel idx: %d, point id: %d, z: %f, max_z: %f, eq: %d \n",
        //        idx, i, pz, max_height_data[idx], 
        //        static_cast<int>(pz == max_height_data[idx]));
        if (pz == max_height_data[idx]) {
            top_intensity_data[idx] = pi;
        }
    }
}

template<typename Dtype>
__global__ void HeightBinKernel(const int n, const int map_size,
    const float height_bin_min_height, const float height_bin_voxel_size,
    const int height_bin_dim, const base::PointF* pc_gpu,
    const int* point2grid, Dtype* height_bin_data) {
    CUDA_KERNEL_LOOP(i, n) {
        int idx = point2grid[i];
        if (idx == -1) {
            continue;
        }
        Dtype pz = pc_gpu[i].z;
        int height_bin_index = static_cast<int>(
            (pz - height_bin_min_height) / height_bin_voxel_size);
        height_bin_index = height_bin_index < 0 ? 0 : height_bin_index;
        height_bin_index = height_bin_index >= height_bin_dim ?
            (height_bin_dim - 1) : height_bin_index;
        height_bin_data[height_bin_index * map_size + idx] = 1.f;
    }
}

__global__ void Point2GridKernel(const base::PointF* pc_gpu,
    const int cloud_size, const float x_min_range, const float y_min_range,
    const float z_min_range, const float z_max_range, const float voxel_x_size,
    const float voxel_y_size, const int grid_x_size, const int grid_y_size,
    const bool enable_rotate_45degree, int* point2grid) {
    CUDA_KERNEL_LOOP(i, cloud_size) {
        point2grid[i] = -1;
        float px = pc_gpu[i].x;
        float py = pc_gpu[i].y;
        float pz = pc_gpu[i].z;
        if (pz < z_min_range || pz > z_max_range) {
            continue;
        }
        if (enable_rotate_45degree) {
            px = 0.707107 * pc_gpu[i].x - 0.707107 * pc_gpu[i].y;
            py = 0.707107 * pc_gpu[i].x + 0.707107 * pc_gpu[i].y;
        }
        float pos_xf = (px - x_min_range) / voxel_x_size;
        float pos_yf = (py - y_min_range) / voxel_y_size;
        int pos_x = static_cast<int>(pos_xf);
        int pos_y = static_cast<int>(pos_yf);
        if (pos_yf < 0 || pos_y >= grid_y_size ||
            pos_xf < 0 || pos_x >= grid_x_size) {
            continue;
        }
        point2grid[i] = pos_y * grid_x_size + pos_x;
    }
}

template<typename Dtype>
__global__ void PointCloudSumKernel(const base::PointF* pc_gpu,
    const int cloud_size, const int map_size,
    const bool enable_rotate_45degree, const int* point2grid,
    int* grid2pointnum_data, Dtype* voxels_cluster_data) {
    CUDA_KERNEL_LOOP(i, cloud_size) {
        int idx = point2grid[i];
        if (idx == -1) {
            continue;
        }
        atomicAdd(&grid2pointnum_data[idx], 1);

        float px = pc_gpu[i].x;
        float py = pc_gpu[i].y;
        if (enable_rotate_45degree) {
            px = 0.707107 * pc_gpu[i].x - 0.707107 * pc_gpu[i].y;
            py = 0.707107 * pc_gpu[i].x + 0.707107 * pc_gpu[i].y;
        }
        // compute sum x,y,z and point-number
        atomicAdd(&voxels_cluster_data[map_size * 0 + idx], px);
        atomicAdd(&voxels_cluster_data[map_size * 1 + idx], py);
        atomicAdd(&voxels_cluster_data[map_size * 2 + idx], pc_gpu[i].z);
    }
}

template<typename Dtype>
__global__ void VoxelFeatureKernel(const int cloud_size, const int map_size,
    const int grid_x_size, const float voxel_x_size, const float voxel_y_size,
    const float x_offset, const float y_offset, const float max_x_range,
    const float max_y_range, const float max_z_range,
    const int voxel_feature_dim, const bool use_input_norm,
    const bool enable_rotate_45degree, const base::PointF* pc_gpu,
    const int* point2grid, const int* grid2pointnum_data,
    Dtype* voxels_cluster_data, Dtype* voxels_data) {
    CUDA_KERNEL_LOOP(i, cloud_size) {
        int grid_idx = point2grid[i];
        if (grid_idx == -1) {
            continue;
        }
        float point_num = max(1.f, static_cast<float>(grid2pointnum_data[grid_idx]));
        float x_mean = voxels_cluster_data[map_size * 0 + grid_idx] / point_num;
        float y_mean = voxels_cluster_data[map_size * 1 + grid_idx] / point_num;
        float z_mean = voxels_cluster_data[map_size * 2 + grid_idx] / point_num;

        int coord_y = grid_idx / grid_x_size;
        int coord_x = grid_idx % grid_x_size;
        float px = pc_gpu[i].x;
        float py = pc_gpu[i].y;
        if (enable_rotate_45degree) {
            px = 0.707107 * pc_gpu[i].x - 0.707107 * pc_gpu[i].y;
            py = 0.707107 * pc_gpu[i].x + 0.707107 * pc_gpu[i].y;
        }
        voxels_data[i * voxel_feature_dim + 4] = px - x_mean;
        voxels_data[i * voxel_feature_dim + 5] = py - y_mean;
        voxels_data[i * voxel_feature_dim + 6] = pc_gpu[i].z - z_mean;
        voxels_data[i * voxel_feature_dim + 7] = Dtype(px) -
            (static_cast<Dtype>(coord_x) * voxel_x_size + x_offset);
        voxels_data[i * voxel_feature_dim + 8] = Dtype(py) -
            (static_cast<Dtype>(coord_y) * voxel_y_size + y_offset);
        if (use_input_norm) {
            voxels_data[i * voxel_feature_dim + 0] = px / max_x_range;
            voxels_data[i * voxel_feature_dim + 1] = py / max_y_range;
            voxels_data[i * voxel_feature_dim + 2] = pc_gpu[i].z / max_z_range;
            voxels_data[i * voxel_feature_dim + 3] = pc_gpu[i].intensity / 255.0;
        } else {
            voxels_data[i * voxel_feature_dim + 0] = px;
            voxels_data[i * voxel_feature_dim + 1] = py;
            voxels_data[i * voxel_feature_dim + 2] = pc_gpu[i].z;
            voxels_data[i * voxel_feature_dim + 3] = pc_gpu[i].intensity;
        }
    }
}

__global__ void CanvasFeatureKernel(const int n, const int pillar_feature_dim, const int map_size,
    const int grid_x_size, const int* point2grid,
    // float* pillar_feature_data, float* canvas_feature_data) {
    const float* pillar_feature_data, float* canvas_feature_data) {
    CUDA_KERNEL_LOOP(i, n) {
        int grid_idx = point2grid[i];
        if (grid_idx == -1) {
            continue;
        }
        int y = grid_idx / grid_x_size;
        int x = grid_idx % grid_x_size;
        for (int c_idx = 0; c_idx < pillar_feature_dim; c_idx++) {
            int canvas_idx = c_idx * map_size + y * grid_x_size + x;
            int pillar_idx = i * pillar_feature_dim + c_idx;
            atomicMax(&canvas_feature_data[canvas_idx],
                pillar_feature_data[pillar_idx]);
        }
    }
}

__global__ void InitCanvasFeatureKernel(const int n, const int cloud_size,
    const int voxel_feature_dim, const int map_size, const int grid_x_size,
    const int max_points_in_voxel, const int* grid2pointnum_data, 
    const float* pillar_feature_data, float* canvas_feature_data) {
    CUDA_KERNEL_LOOP(i, n) {
        int grid_idx = i % map_size;
        int channel_idx = i / map_size;
        int point_num = grid2pointnum_data[grid_idx];
        int y = grid_idx / grid_x_size;
        int x = grid_idx % grid_x_size;
        int canvas_idx = channel_idx * map_size + y * grid_x_size + x;
        if (point_num == 0 || point_num >= max_points_in_voxel) {
            canvas_feature_data[canvas_idx] = 0;
        } else {
            int pillar_idx = cloud_size * voxel_feature_dim + channel_idx;
            canvas_feature_data[canvas_idx] = pillar_feature_data[pillar_idx];
        }
    }
}

template <typename Dtype>
__global__ void SetKernel(const int n, const Dtype alpha, Dtype* y) {
    CUDA_KERNEL_LOOP(i, n) {
        y[i] = alpha;
    }
}

// rotate overlap
__global__ void ComputeRotateOverlapKernel(const int nthreads,
    const int box_block_size, const int box_num,
    const float nms_overlap_thresh, const float* box3d,
    const float* box_corners, const int* sorted_idx,
    bool* rotate_overlapped_data) {
    CUDA_KERNEL_LOOP(idx, nthreads) {  
        const int j = idx % box_num;
        const int i = idx / box_num;
        if (i == j) {
            // Ignore same bbox.
            rotate_overlapped_data[idx] = 0;
            return;
        }
        // Compute overlap between i-th bbox and j-th bbox.
        int start_loc_i = sorted_idx[i] * box_block_size;
        int start_loc_j = sorted_idx[j] * box_block_size;
        float area1 = box3d[start_loc_i + 3] * box3d[start_loc_i + 4];
        float area2 = box3d[start_loc_j + 3] * box3d[start_loc_j + 4];
        // printf("x: %.5f, y: %.5f, w: %.5f, l: %.5f\n", box3d[start_loc_i],
        //  box3d[start_loc_i + 1], box3d[start_loc_i + 3],
        //  box3d[start_loc_i + 4]);

        start_loc_i = sorted_idx[i] * 8;
        start_loc_j = sorted_idx[j] * 8;
        float area_inter = rotate_inter_gpu(box_corners + start_loc_i,
                            box_corners + start_loc_j);
        float overlap = area_inter / max(area1 + area2 - area_inter, 1e-8);
        // printf("rotate overlap %.5f, i: %d, j: %d\n", overlap, i, j);
        rotate_overlapped_data[idx] = overlap > nms_overlap_thresh;
    }
}

__global__ void TransBoxforNmsKernel(const int box_num, const float quantize,
    const float* res_box_data, float* box_corner, float* box_for_nms,
    float length_enlarge, float width_enlarge) {
    CUDA_KERNEL_LOOP(idx, box_num) {
        float x = res_box_data[idx * kBoxBlockSize + 0];
        float y = res_box_data[idx * kBoxBlockSize + 1];
        float l = res_box_data[idx * kBoxBlockSize + 3];
        float w = res_box_data[idx * kBoxBlockSize + 4];
        float r = res_box_data[idx * kBoxBlockSize + 6];
        if (quantize > 0) {
            // printf(": %.5f/%.5f\n", w, ceil(w / quantize) * quantize);
            w = ceil(w / quantize) * quantize;
            l = ceil(l / quantize) * quantize;
        }
        if (length_enlarge > 0) {
            l = l + length_enlarge;
        }
        if (width_enlarge > 0) {
            w = w + width_enlarge;
        }

        float cos_r = cos(r);
        float sin_r = sin(r);
        float hl = l * 0.5;
        float hw = w * 0.5;

        float x1 = (-hl) * cos_r - (-hw) * sin_r + x;
        float y1 = (-hl) * sin_r + (-hw) * cos_r + y;
        float x2 = ( hl) * cos_r - (-hw) * sin_r + x;
        float y2 = ( hl) * sin_r + (-hw) * cos_r + y;
        float x3 = ( hl) * cos_r - ( hw) * sin_r + x;
        float y3 = ( hl) * sin_r + ( hw) * cos_r + y;
        float x4 = (-hl) * cos_r - ( hw) * sin_r + x;
        float y4 = (-hl) * sin_r + ( hw) * cos_r + y;

        box_corner[idx * 8 + 0] = x1;
        box_corner[idx * 8 + 1] = y1;
        box_corner[idx * 8 + 2] = x2;
        box_corner[idx * 8 + 3] = y2;
        box_corner[idx * 8 + 4] = x3;
        box_corner[idx * 8 + 5] = y3;
        box_corner[idx * 8 + 6] = x4;
        box_corner[idx * 8 + 7] = y4;

        box_for_nms[idx * 4 + 0] = min(min(min(x1, x2), x3), x4);
        box_for_nms[idx * 4 + 1] = min(min(min(y1, y2), y3), y4);
        box_for_nms[idx * 4 + 2] = max(max(max(x1, x2), x3), x4);
        box_for_nms[idx * 4 + 3] = max(max(max(y1, y2), y3), y4);
    }
}

__global__ void ComputeOverlapKernel(const int nthreads, const int box_num,
    const int box_size, const float* box_for_nms,
    const int* box_indices, float* overlapped_data) {
  CUDA_KERNEL_LOOP(idx, nthreads) {  
        const int j = idx % box_num;
        const int i = idx / box_num;
        if (i == j) {
            // Ignore same bbox.
            overlapped_data[idx] = 0.f;
            return;
        }
        // Compute overlap between i-th bbox and j-th bbox.
        const int start_loc_i = box_indices[i] * box_size;
        const int start_loc_j = box_indices[j] * box_size;
        float overlap = jaccard_overlap_gpu(box_for_nms + start_loc_i,
                            box_for_nms + start_loc_j);
        overlapped_data[idx] = overlap;
    }
}

__global__ void Points2BoxidKernel(const int cloud_size,
    const bool enable_rotate_45degree, const int box_num,
    const base::PointF* pc, const int* kept_indices, const float* res_box_data,
    const float* box_corner, const float* box_for_nms,
    int* valid_point_num_data, int* valid_point2boxid_data,
    int* valid_point_indices_data, float bottom_enlarge, float top_enlarge) {
    CUDA_KERNEL_LOOP(point_idx, cloud_size) {
        float px = pc[point_idx].x;
        float py = pc[point_idx].y;
        float pz = pc[point_idx].z;
        if (enable_rotate_45degree) {
            px = 0.707107 * pc[point_idx].x - 0.707107 * pc[point_idx].y;
            py = 0.707107 * pc[point_idx].x + 0.707107 * pc[point_idx].y;
        }

        for (int i = 0; i < box_num; i++) {
            const int box_idx = kept_indices[i];
            // inside standup box
            if (px < box_for_nms[box_idx * 4 + 0] ||
                px > box_for_nms[box_idx * 4 + 2] ||
                py < box_for_nms[box_idx * 4 + 1] ||
                py > box_for_nms[box_idx * 4 + 3]) {
                continue;
            }
            // inside in top z & bottom z
            float z = res_box_data[box_idx * kBoxBlockSize + 2];
            float h = res_box_data[box_idx * kBoxBlockSize + 5];
            if (pz < (z - h / 2 - bottom_enlarge) ||
                pz > (z + h / 2 + top_enlarge)) {
                continue;
            }
            // inside x y box
            float x1 = box_corner[box_idx * 8 + 0];
            float x2 = box_corner[box_idx * 8 + 2];
            float x3 = box_corner[box_idx * 8 + 4];
            float x4 = box_corner[box_idx * 8 + 6];
            float y1 = box_corner[box_idx * 8 + 1];
            float y2 = box_corner[box_idx * 8 + 3];
            float y3 = box_corner[box_idx * 8 + 5];
            float y4 = box_corner[box_idx * 8 + 7];
    
            double angle1 = (px - x1) * (x2 - x1) + (py - y1) * (y2 - y1);
            double angle2 = (px - x2) * (x3 - x2) + (py - y2) * (y3 - y2);
            double angle3 = (px - x3) * (x4 - x3) + (py - y3) * (y4 - y3);
            double angle4 = (px - x4) * (x1 - x4) + (py - y4) * (y1 - y4);
            if ((angle1 < 0 && angle2 < 0 && angle3 < 0 && angle4 < 0) ||
                (angle1 > 0 && angle2 > 0 && angle3 > 0 && angle4 > 0)) {
                // printf("point_idx: %d, box_idx: %d\n", point_idx, box_idx);
                // atomicAdd(valid_point_num_data, 1);
                // point2boxid_data[point_idx] = i;
                int count = atomicAdd(valid_point_num_data, 1);
                valid_point_indices_data[count] = point_idx;
                valid_point2boxid_data[count] = i;
            }
        }
    }
}

__global__ void SimpleTransBoxKernel(const int box_num, const float quantize,
    const int* kept_indices, const float* res_box_data, float* box_corner, float* box_for_nms,
    float length_enlarge, float width_enlarge) {
    CUDA_KERNEL_LOOP(idx, box_num) {
        const int box_idx = kept_indices[idx];
        float x = res_box_data[box_idx * kBoxBlockSize + 0];
        float y = res_box_data[box_idx * kBoxBlockSize + 1];
        float l = res_box_data[box_idx * kBoxBlockSize + 3];
        float w = res_box_data[box_idx * kBoxBlockSize + 4];
        float r = res_box_data[box_idx * kBoxBlockSize + 6];
        if (quantize > 0) {
            // printf(": %.5f/%.5f\n", w, ceil(w / quantize) * quantize);
            w = ceil(w / quantize) * quantize;
            l = ceil(l / quantize) * quantize;
        }
        if (length_enlarge > 0) {
            l = l + length_enlarge;
        }
        if (width_enlarge > 0) {
            w = w + width_enlarge;
        }

        float cos_r = cos(r);
        float sin_r = sin(r);
        float hl = l * 0.5;
        float hw = w * 0.5;

        float x1 = (-hl) * cos_r - (-hw) * sin_r + x;
        float y1 = (-hl) * sin_r + (-hw) * cos_r + y;
        float x2 = ( hl) * cos_r - (-hw) * sin_r + x;
        float y2 = ( hl) * sin_r + (-hw) * cos_r + y;
        float x3 = ( hl) * cos_r - ( hw) * sin_r + x;
        float y3 = ( hl) * sin_r + ( hw) * cos_r + y;
        float x4 = (-hl) * cos_r - ( hw) * sin_r + x;
        float y4 = (-hl) * sin_r + ( hw) * cos_r + y;

        box_corner[idx * 8 + 0] = x1;
        box_corner[idx * 8 + 1] = y1;
        box_corner[idx * 8 + 2] = x2;
        box_corner[idx * 8 + 3] = y2;
        box_corner[idx * 8 + 4] = x3;
        box_corner[idx * 8 + 5] = y3;
        box_corner[idx * 8 + 6] = x4;
        box_corner[idx * 8 + 7] = y4;

        box_for_nms[idx * 4 + 0] = min(min(min(x1, x2), x3), x4);
        box_for_nms[idx * 4 + 1] = min(min(min(y1, y2), y3), y4);
        box_for_nms[idx * 4 + 2] = max(max(max(x1, x2), x3), x4);
        box_for_nms[idx * 4 + 3] = max(max(max(y1, y2), y3), y4);
    }
}

__global__ void SimplePoints2BoxKernel(const int cloud_size,
    const bool enable_rotate_45degree, const int box_num,
    const base::PointF* pc, const int* kept_indices, const float* res_box_data,
    const float* box_corner, const float* box_for_nms,
    int* valid_point_num_data, int* valid_point2boxid_data,
    int* valid_point_indices_data, float bottom_enlarge, float top_enlarge) {
    CUDA_KERNEL_LOOP(point_idx, cloud_size) {
        float px = pc[point_idx].x;
        float py = pc[point_idx].y;
        float pz = pc[point_idx].z;
        if (enable_rotate_45degree) {
            px = 0.707107 * pc[point_idx].x - 0.707107 * pc[point_idx].y;
            py = 0.707107 * pc[point_idx].x + 0.707107 * pc[point_idx].y;
        }

        for (int i = 0; i < box_num; i++) {
            const int box_idx = kept_indices[i];
            // inside standup box
            if (px < box_for_nms[i * 4 + 0] || px > box_for_nms[i * 4 + 2] ||
                py < box_for_nms[i * 4 + 1] || py > box_for_nms[i * 4 + 3]) {
                continue;
            }
            // inside in top z & bottom z
            float z = res_box_data[box_idx * kBoxBlockSize + 2];
            float h = res_box_data[box_idx * kBoxBlockSize + 5];
            if (pz < (z - h / 2 - bottom_enlarge) || pz > (z + h / 2 + top_enlarge)) {
                continue;
            }
            // inside x y box
            float x1 = box_corner[i * 8 + 0];
            float x2 = box_corner[i * 8 + 2];
            float x3 = box_corner[i * 8 + 4];
            float x4 = box_corner[i * 8 + 6];
            float y1 = box_corner[i * 8 + 1];
            float y2 = box_corner[i * 8 + 3];
            float y3 = box_corner[i * 8 + 5];
            float y4 = box_corner[i * 8 + 7];
    
            double angle1 = (px - x1) * (x2 - x1) + (py - y1) * (y2 - y1);
            double angle2 = (px - x2) * (x3 - x2) + (py - y2) * (y3 - y2);
            double angle3 = (px - x3) * (x4 - x3) + (py - y3) * (y4 - y3);
            double angle4 = (px - x4) * (x1 - x4) + (py - y4) * (y1 - y4);
            if ((angle1 < 0 && angle2 < 0 && angle3 < 0 && angle4 < 0) ||
                (angle1 > 0 && angle2 > 0 && angle3 > 0 && angle4 > 0)) {
                int count = atomicAdd(valid_point_num_data, 1);
                valid_point_indices_data[count] = point_idx;
                valid_point2boxid_data[count] = i;
            }
        }
    }
}

__global__ void GetAllRemainBoxInfoKernel(const int nthreads,
    const int all_res_num, const float* res_box_data,
    const float* res_conf_data, const int* res_cls_data,
    float* all_res_box_data, float* all_res_conf_data,
    int* all_res_cls_data) {
    CUDA_KERNEL_LOOP(idx, nthreads) {
        all_res_box_data[(all_res_num + idx) * kBoxBlockSize + 0] =
            res_box_data[idx * kBoxBlockSize + 0];
        all_res_box_data[(all_res_num + idx) * kBoxBlockSize + 1] =
            res_box_data[idx * kBoxBlockSize + 1];
        all_res_box_data[(all_res_num + idx) * kBoxBlockSize + 2] =
            res_box_data[idx * kBoxBlockSize + 2];
        all_res_box_data[(all_res_num + idx) * kBoxBlockSize + 3] =
            res_box_data[idx * kBoxBlockSize + 3];
        all_res_box_data[(all_res_num + idx) * kBoxBlockSize + 4] =
            res_box_data[idx * kBoxBlockSize + 4];
        all_res_box_data[(all_res_num + idx) * kBoxBlockSize + 5] =
            res_box_data[idx * kBoxBlockSize + 5];
        all_res_box_data[(all_res_num + idx) * kBoxBlockSize + 6] =
            res_box_data[idx * kBoxBlockSize + 6];

        all_res_conf_data[all_res_num + idx] = res_conf_data[idx];
        all_res_cls_data[all_res_num + idx] = res_cls_data[idx];
    }
}

__global__ void DecodeObjectKernel(const int map_size,
    const int nms_pre_max_size, const float min_x_range,
    const float min_y_range, const float voxel_x_size,
    const float voxel_y_size, const int head_x_size, const int head_y_size,
    const int downsample_size, const int num_classes_in_task,
    const int cls_range, const float* score_thresh,
    const float* reg, const float* hei, const float* dim, const float* cls,
    const float* rot, int* res_box_num_data, float* res_box_data,
    float* res_conf_data, int* res_cls_data) {
    CUDA_KERNEL_LOOP(idx, map_size) {
        float max_score = cls[idx];
        int label = cls_range;
        for (int i = 1; i < num_classes_in_task; ++i) {
            float cur_score = cls[idx + i * map_size];
            if (cur_score > max_score) {
                max_score = cur_score;
                label = i + cls_range;
            }
        }

        int coor_x = idx % head_x_size;
        int coor_y = idx / head_x_size;
        float conf = sigmoid_gpu(max_score);
        if (conf > score_thresh[label]) {
            int cur_valid_box_id = atomicAdd(res_box_num_data, 1);
            // printf("curr conf: %.2f, label: %d, box_num: %d\n", conf, label, res_box_num_data[0]);
            // Fixme:
            //    if the quantity of valid box is too large, there is a risk of box loss
            if (cur_valid_box_id < nms_pre_max_size) {
                res_box_data[cur_valid_box_id * kBoxBlockSize + 0] = 
                    (reg[idx + 0 * map_size] + coor_x) * downsample_size * voxel_x_size + min_x_range;
                res_box_data[cur_valid_box_id * kBoxBlockSize + 1] = 
                    (reg[idx + 1 * map_size] + coor_y) * downsample_size * voxel_y_size + min_y_range;
                res_box_data[cur_valid_box_id * kBoxBlockSize + 2] = hei[idx];
                res_box_data[cur_valid_box_id * kBoxBlockSize + 3] = expf(dim[idx + 0 * map_size]);
                res_box_data[cur_valid_box_id * kBoxBlockSize + 4] = expf(dim[idx + 1 * map_size]);
                res_box_data[cur_valid_box_id * kBoxBlockSize + 5] = expf(dim[idx + 2 * map_size]);
                res_box_data[cur_valid_box_id * kBoxBlockSize + 6] = atan2f(rot[idx], rot[idx + map_size]);
                res_conf_data[cur_valid_box_id] = conf;
                res_cls_data[cur_valid_box_id] = label;
                // printf("conf: %f, label: %d", conf, label); 
            }
            // printf("AA: %d", a);
            // printf("box_number: %d\n", cur_valid_box_id);
        }
    }
}

void CPDetection::GeneratePfnFeatureGPU() {
    float* canvas_feature_data = canvas_feature_blob_->mutable_gpu_data();
    float* voxels_cluster_data = canvas_feature_data + feature_offset_["canvas_feature"];
    int* grid2pointnum_data = grid2pointnum_blob_->mutable_gpu_data();
    
    BASE_GPU_CHECK(cudaMemsetAsync(voxels_cluster_data, 0.f, sizeof(float) * 3 * grid_x_size_ * grid_y_size_, stream_));
    BASE_GPU_CHECK(cudaMemsetAsync(grid2pointnum_data, 0, sizeof(int) * grid_x_size_ * grid_y_size_, stream_));

    int cloud_size = cur_cloud_ptr_->size();
    // pc_gpu
    if (pc_gpu_ != nullptr) {
        BASE_GPU_CHECK(cudaFree(pc_gpu_));  // free cuda memory
    }
    // memory malloc and copy from cur_cloud_ptr_ to pc_gpu_
    BASE_GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&pc_gpu_), cloud_size * sizeof(base::PointF)));
    BASE_GPU_CHECK(cudaMemcpy(pc_gpu_, &(cur_cloud_ptr_->front()), sizeof(base::PointF) * cloud_size, cudaMemcpyHostToDevice));

    // init output blob
    point2grid_blob_->Reshape(std::vector<int>{cloud_size});
    int* point2grid_data = point2grid_blob_->mutable_gpu_data();
    BASE_GPU_CHECK(cudaMemsetAsync(point2grid_data, 0, sizeof(int) * cloud_size, stream_));

    voxels_blob_->Reshape(std::vector<int>{cloud_size, 1, voxel_feature_dim_, 1});
    float* voxel_gpu = voxels_blob_->mutable_gpu_data();
    BASE_GPU_CHECK(cudaMemsetAsync(voxel_gpu, 0.f, sizeof(float) * cloud_size * voxel_feature_dim_, stream_));

    // x,y,z means. shape: (4, pillar_num), 4: [x, y, z, point_number]
    // float* xyzs = nullptr;
    // BASE_GPU_CHECK(cudaMalloc((void**)&xyzs, sizeof(float) * grid_x_size_ * grid_y_size_ * 4));
    // BASE_GPU_CHECK(cudaMemsetAsync(xyzs, 0.0f, sizeof(float) * grid_x_size_ * grid_y_size_ * 4, stream_));

    int block_size = DIVUP(cloud_size, kGPUThreadSize);
    Point2GridKernel<<<block_size, kGPUThreadSize, 0, stream_>>>(pc_gpu_,
        cloud_size, x_min_range_, y_min_range_, z_min_range_, z_max_range_,
        voxel_x_size_, voxel_y_size_, grid_x_size_, grid_y_size_,
        enable_rotate_45degree_, point2grid_data);
    cudaDeviceSynchronize();
    // cudaError_t err1 = cudaGetLastError();
    // if (err1 != cudaSuccess) {
    //     AERROR << "[cudaError]: " << cudaGetErrorString(err1);
    // }
    
    if (use_cnnseg_features_) {
        float* max_height_data = canvas_feature_data + feature_offset_["max_height"];
        float* mean_height_data = canvas_feature_data + feature_offset_["mean_height"];
        float* top_intensity_data = canvas_feature_data + feature_offset_["top_intensity"];
        float* mean_intensity_data = canvas_feature_data +
            feature_offset_["mean_intensity"];
        float* count_data = canvas_feature_data +
            feature_offset_["count"];
        float* nonempty_data = canvas_feature_data +
            feature_offset_["nonempty"];
        float* height_bin_data = canvas_feature_data +
            feature_offset_["height_bin"];

        BASE_GPU_CHECK(cudaMemsetAsync(max_height_data, -5.f, 
            sizeof(float) * map_size_, stream_));
        BASE_GPU_CHECK(cudaMemsetAsync(mean_height_data, 0.f,
            sizeof(float) * map_size_, stream_));
        BASE_GPU_CHECK(cudaMemsetAsync(top_intensity_data, 0.f,
            sizeof(float) * map_size_, stream_));
        BASE_GPU_CHECK(cudaMemsetAsync(mean_intensity_data, 0.f,
            sizeof(float) * map_size_, stream_));
        BASE_GPU_CHECK(cudaMemsetAsync(count_data, 0.f,
            sizeof(float) * map_size_, stream_));
        BASE_GPU_CHECK(cudaMemsetAsync(nonempty_data, 0.f,
            sizeof(float) * map_size_, stream_));
        BASE_GPU_CHECK(cudaMemsetAsync(height_bin_data, 0.f,
            sizeof(float) * map_size_ * height_bin_dim_, stream_));
    
        // cnnseg features: max_height, mean_height, top_intensity
        // mean_intensity, count, nonempty
        MapKernel<float><<<block_size, kGPUThreadSize, 0, stream_>>>(
            cloud_size, pc_gpu_, point2grid_blob_->gpu_data(), max_height_data,
            mean_height_data, mean_intensity_data, count_data);
        cudaDeviceSynchronize();

        TopIntensityKernel<float><<<block_size, kGPUThreadSize, 0, stream_>>>(
            cloud_size, pc_gpu_, point2grid_blob_->gpu_data(),
            top_intensity_data, max_height_data);
        cudaDeviceSynchronize();
        
        AverageKernel<float><<<block_size, kGPUThreadSize, 0, stream_>>>(
            map_size_, count_data, max_height_data, mean_height_data,
            mean_intensity_data, nonempty_data);
        cudaDeviceSynchronize();
    
        // height bin feature
        HeightBinKernel<float><<<block_size, kGPUThreadSize, 0, stream_>>>(
            cloud_size, map_size_, height_bin_min_height_,
            height_bin_voxel_size_, height_bin_dim_,
            pc_gpu_, point2grid_blob_->gpu_data(), height_bin_data);
        cudaDeviceSynchronize();
    }

    PointCloudSumKernel<float><<<block_size, kGPUThreadSize, 0, stream_>>>(pc_gpu_,
        cloud_size, map_size_, enable_rotate_45degree_,
        point2grid_blob_->gpu_data(), grid2pointnum_data, voxels_cluster_data);
    cudaDeviceSynchronize();
    // cudaError_t err2 = cudaGetLastError();
    // if (err2 != cudaSuccess) {
        // AERROR << "[cudaError]: " << cudaGetErrorString(err2);
    // }

    VoxelFeatureKernel<float><<<block_size, kGPUThreadSize, 0, stream_>>>(
        cloud_size, map_size_, grid_x_size_, voxel_x_size_, voxel_y_size_,
        x_offset_, y_offset_, x_max_range_, y_max_range_, z_max_range_,
        voxel_feature_dim_, use_input_norm_, enable_rotate_45degree_, pc_gpu_,
        point2grid_blob_->gpu_data(), grid2pointnum_blob_->gpu_data(),
        voxels_cluster_data, voxel_gpu);
    cudaDeviceSynchronize();
    // cudaError_t err3 = cudaGetLastError();
    // if (err3 != cudaSuccess) {
        // AERROR << "[cudaError]: " << cudaGetErrorString(err3);
    // }
  
    // DEVICE_SAVE<float>(voxel_gpu, cloud_size, voxel_feature_dim_, std::to_string(lidar_frame_ref_->timestamp) + "_voxel_feature.txt");

    // output
    pfn_pillar_feature_blob_->Reshape({static_cast<int>(cloud_size), pillar_feature_dim_});
    // float* pfn_data = pfn_pillar_feature_blob_->mutable_gpu_data();
    // BASE_GPU_CHECK(cudaMemsetAsync(pfn_data, 0.f, sizeof(float) * cloud_size * pillar_feature_dim_, stream_));
    // DEVICE_SAVE<float>(pfn_pillar_feature_blob_->gpu_data(), cloud_size, pillar_feature_dim_, std::to_string(lidar_frame_ref_->timestamp) + "_pfn1_feature.txt");
}

void CPDetection::GenerateBackboneFeatureGPU(const base::Blob<float>* pillar_feature_blob) {
    const float *pillar_feature_data = pillar_feature_blob->gpu_data();
    const int* point2grid_data = point2grid_blob_->gpu_data();
    const int* grid2pointnum_data = grid2pointnum_blob_->gpu_data();

    // DEVICE_SAVE<float>(pillar_feature_data, cloud_size_, pillar_feature_dim_, std::to_string(lidar_frame_ref_->timestamp) + "_pfn_feature.txt");
    // DEVICE_SAVE<int>(point2grid_blob_->gpu_data(), cloud_size_, 1, std::to_string(lidar_frame_ref_->timestamp) + "_p2g_feature.txt");

    // input
    int shape = pillar_feature_dim_ + cnnseg_feature_dim_;
    canvas_feature_blob_->Reshape({1, shape, grid_x_size_, grid_y_size_});
    float* canvas_feature_data = canvas_feature_blob_->mutable_gpu_data() +
        feature_offset_["canvas_feature"];
    BASE_GPU_CHECK(cudaMemsetAsync(canvas_feature_data, 0.f, sizeof(float) * map_size_ * pillar_feature_dim_, stream_));

    int cloud_size = cur_cloud_ptr_->size();
    int block_size = DIVUP(cloud_size, kGPUThreadSize);
    CanvasFeatureKernel<<<block_size, kGPUThreadSize, 0, stream_>>>(
        cloud_size, pillar_feature_dim_, map_size_, grid_x_size_,
        point2grid_data, pillar_feature_data, canvas_feature_data);
    cudaDeviceSynchronize();
    // cudaError_t err = cudaGetLastError();
    // if (err != cudaSuccess) {
        // AERROR << "[cudaError]: " << cudaGetErrorString(err);
    // }

    // DEVICE_SAVE<float>(canvas_feature_data, shape, map_size_, std::to_string(lidar_frame_ref_->timestamp) + "_canvas1_feature.txt", true);
    // DEVICE_SAVE<float>(canvas_feature_blob_->gpu_data(), shape, map_size_, std::to_string(lidar_frame_ref_->timestamp) + "_canvas_feature.txt", true);
    // output
    output_box_blob_->Reshape({1, 6 * num_tasks_, head_x_size_, head_y_size_});
    output_cls_blob_->Reshape({1, num_classes_, head_x_size_, head_y_size_});
    output_dir_blob_->Reshape({1, 2 * num_tasks_, head_x_size_, head_y_size_});
}

void CPDetection::AssignPoints2Boxid(const std::vector<int>& kept_indices) {
    int all_res_box_num = all_res_box_blob_->shape(0);
    box_corner_blob_->Reshape(std::vector<int>{all_res_box_num, 8});
    box_for_nms_blob_->Reshape(std::vector<int>{all_res_box_num, 4});
    float* box_corner = box_corner_blob_->mutable_gpu_data();
    float* box_for_nms = box_for_nms_blob_->mutable_gpu_data();
    int block_size = DIVUP(all_res_box_num, kGPUThreadSize);
    TransBoxforNmsKernel<<<block_size, kGPUThreadSize, 0, stream_>>>(all_res_box_num,
        0.2f, all_res_box_blob_->gpu_data(), box_corner, box_for_nms,
        length_enlarge_value_, width_enlarge_value_);
    
    // cal point2boxid
    int box_num = static_cast<int>(kept_indices.size());
    kept_indices_blob_->Reshape(std::vector<int>{box_num});

    valid_point_num_blob_->Reshape(std::vector<int>{total_cloud_size_});    
    int* valid_point_num_data = valid_point_num_blob_->mutable_gpu_data();
    int* valid_point_indices_data = valid_point_indices_blob_->mutable_gpu_data();
    int* valid_point2boxid_data = valid_point2boxid_blob_->mutable_gpu_data();
    int* kept_indices_data = kept_indices_blob_->mutable_gpu_data();

    BASE_GPU_CHECK(cudaMemsetAsync(valid_point_num_data, 0, sizeof(int), stream_));
    BASE_GPU_CHECK(cudaMemsetAsync(valid_point_indices_data, -1, sizeof(int) * total_cloud_size_, stream_));
    BASE_GPU_CHECK(cudaMemsetAsync(valid_point2boxid_data, -1, sizeof(int) * total_cloud_size_, stream_));
    BASE_GPU_CHECK(cudaMemcpyAsync(kept_indices_data, &kept_indices[0],
        sizeof(int) * box_num, cudaMemcpyHostToDevice, stream_));

    base::PointF* pc_gpu = nullptr;
    if (pc_gpu != nullptr) {
        BASE_GPU_CHECK(cudaFree(pc_gpu));  // free cuda memory
    }
    // memory malloc and copy from original_cloud_ to pc_gpu
    BASE_GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&pc_gpu),
        total_cloud_size_ * sizeof(base::PointF)));
    BASE_GPU_CHECK(cudaMemcpy(pc_gpu, &(original_cloud_->front()),
        sizeof(base::PointF) * total_cloud_size_, cudaMemcpyHostToDevice));
    block_size = DIVUP(total_cloud_size_, kGPUThreadSize);
    Points2BoxidKernel<<<block_size, kGPUThreadSize, 0, stream_>>>(total_cloud_size_,
        enable_rotate_45degree_, box_num, pc_gpu, kept_indices_data,
        all_res_box_blob_->gpu_data(), box_corner_blob_->gpu_data(),
        box_for_nms_blob_->gpu_data(), valid_point_num_data,
        valid_point2boxid_data, valid_point_indices_data,
        bottom_enlarge_value_, top_enlarge_value_);
    
    BASE_GPU_CHECK(cudaFree(pc_gpu));  // free cuda memory
}

void CPDetection::SimpleAssignPoints2Boxid(const std::vector<int>& kept_indices) {
    // int all_res_box_num = all_res_box_blob_->shape(0);
    // box_corner_blob_->Reshape(std::vector<int>{all_res_box_num, 8});
    // box_for_nms_blob_->Reshape(std::vector<int>{all_res_box_num, 4});
    // float* box_corner = box_corner_blob_->mutable_gpu_data();
    // float* box_for_nms = box_for_nms_blob_->mutable_gpu_data();

    // cal point2boxid
    int box_num = static_cast<int>(kept_indices.size());
    kept_indices_blob_->Reshape(std::vector<int>{box_num});
    int* kept_indices_data = kept_indices_blob_->mutable_gpu_data();
    BASE_GPU_CHECK(cudaMemcpyAsync(kept_indices_data, &kept_indices[0],
        sizeof(int) * box_num, cudaMemcpyHostToDevice, stream_));
    
    box_corners_blob_->Reshape(std::vector<int>{box_num, 8});
    box_rects_blob_->Reshape(std::vector<int>{box_num, 4});
    float* box_corners = box_corners_blob_->mutable_gpu_data();
    float* box_rects = box_rects_blob_->mutable_gpu_data();
    int block_size = DIVUP(box_num, kGPUThreadSize);
    SimpleTransBoxKernel<<<block_size, kGPUThreadSize, 0, stream_>>>(box_num,
        0.2f, kept_indices_data, all_res_box_blob_->gpu_data(), box_corners, box_rects,
        length_enlarge_value_, width_enlarge_value_);
    
    valid_point_num_blob_->Reshape(std::vector<int>{total_cloud_size_});
    int* valid_point_num_data = valid_point_num_blob_->mutable_gpu_data();
    int* valid_point_indices_data = valid_point_indices_blob_->mutable_gpu_data();
    int* valid_point2boxid_data = valid_point2boxid_blob_->mutable_gpu_data();

    BASE_GPU_CHECK(cudaMemsetAsync(valid_point_num_data, 0, sizeof(int), stream_));
    BASE_GPU_CHECK(cudaMemsetAsync(valid_point_indices_data, -1, sizeof(int) * max_valid_point_size_, stream_));
    BASE_GPU_CHECK(cudaMemsetAsync(valid_point2boxid_data, -1, sizeof(int) * max_valid_point_size_, stream_));

    base::PointF* pc_gpu = nullptr;
    if (pc_gpu != nullptr) {
        BASE_GPU_CHECK(cudaFree(pc_gpu));  // free cuda memory
    }
    // memory malloc and copy from original_cloud_ to pc_gpu
    BASE_GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&pc_gpu),
        total_cloud_size_ * sizeof(base::PointF)));
    BASE_GPU_CHECK(cudaMemcpy(pc_gpu, &(original_cloud_->front()),
        sizeof(base::PointF) * total_cloud_size_, cudaMemcpyHostToDevice));
    
    block_size = DIVUP(total_cloud_size_, kGPUThreadSize);
    SimplePoints2BoxKernel<<<block_size, kGPUThreadSize, 0, stream_>>>(total_cloud_size_,
        enable_rotate_45degree_, box_num, pc_gpu, kept_indices_data,
        all_res_box_blob_->gpu_data(), box_corners_blob_->gpu_data(),
        box_rects_blob_->gpu_data(), valid_point_num_data,
        valid_point2boxid_data, valid_point_indices_data,
        bottom_enlarge_value_, top_enlarge_value_);
    
    BASE_GPU_CHECK(cudaFree(pc_gpu));  // free cuda memory
}

void CPDetection::ApplyRotateNms(const bool *rotate_overlapped,
    const int valid_box_num, const int* all_sorted_indices,
    std::vector<int> *box_reverve_flag, bool skip_suppressed) {
    std::vector<bool> suppressed(valid_box_num, false);
    for (int i = 0; i < valid_box_num; ++i) {
        // fprintf(stderr, "apply_nms: %d - ", i);
        // int ind_i = all_sorted_indices[i];
        if (!suppressed[i]) {
            box_reverve_flag->at(i) = 1;
            // fprintf(stderr, "kept\n", i);
        } else {
            // fprintf(stderr, "skip\n", i);
            box_reverve_flag->at(i) = -1;
            if (skip_suppressed) {
                continue;
            }
        }
        const auto overlapped_data = rotate_overlapped + i * valid_box_num;
        for (int j = i + 1; j < valid_box_num; ++j) {
            if (overlapped_data[j]) {
                // printf("apply overlap nms: %d, suppressed: %d, overlap: %.5f \n", i, j, overlapped_data[j]);
                suppressed[j] = true;
            }
        }
    }
}

int CPDetection::ApplyNmsGPU(const int box_num_pre,
    const int all_res_num, std::vector<int>* kept_indices) {
    // switch current gpu device data
    // int gpu_id = base::GetDevice();
    // auto par = thrust::cuda::par(GPUAllocator::Instance().GetAllocator(gpu_id));
    if (box_num_pre <= 0) { 
        return 0; 
    }
    
    box_corner_blob_->Reshape(std::vector<int>{box_num_pre, 8});
    box_for_nms_blob_->Reshape(std::vector<int>{box_num_pre, 4});
    float* box_corner = box_corner_blob_->mutable_gpu_data();
    float* box_for_nms = box_for_nms_blob_->mutable_gpu_data();

    int block_size = DIVUP(box_num_pre, kGPUThreadSize);
    TransBoxforNmsKernel<<<block_size, kGPUThreadSize, 0, stream_>>>(
        box_num_pre, -1.f, res_box_blob_->gpu_data(), box_corner,
        box_for_nms, -1.f, -1.f);
  
    // Sort detections based on score
    remain_conf_blob_->Reshape(std::vector<int>{box_num_pre});
    float* remain_conf_data = remain_conf_blob_->mutable_gpu_data();
    BASE_GPU_CHECK(cudaMemcpyAsync(remain_conf_data,
        res_conf_blob_->gpu_data(), sizeof(float) * box_num_pre,
        cudaMemcpyDeviceToDevice, stream_));
    
    res_sorted_indices_blob_->Reshape(std::vector<int>{box_num_pre});
    int* res_sorted_indices_data = res_sorted_indices_blob_->mutable_gpu_data();

    thrust::sequence(thrust::device, res_sorted_indices_data, res_sorted_indices_data + box_num_pre);
    thrust::sort_by_key(thrust::device, remain_conf_data, remain_conf_data + box_num_pre,
        res_sorted_indices_data, thrust::greater<float>());

    // calculate rotate_overlap
    rotate_overlapped_blob_->Reshape(std::vector<int>{box_num_pre, box_num_pre});
    int nthreads = box_num_pre * box_num_pre;
    block_size = DIVUP(nthreads, kGPUThreadSize);
    ComputeRotateOverlapKernel<<<block_size, kGPUThreadSize, 0, stream_>>>(
        nthreads, kBoxBlockSize, box_num_pre,
        nms_overlap_thresh_, res_box_blob_->gpu_data(),
        box_corner_blob_->gpu_data(), res_sorted_indices_blob_->gpu_data(),
        rotate_overlapped_blob_->mutable_gpu_data());

    BASE_GPU_CHECK(cudaStreamSynchronize(stream_));
    // const int* res_sorted_indices = res_sorted_indices_blob_->cpu_data();
    const int* res_sorted_indices = res_sorted_indices_blob_->cpu_data();
    std::vector<int> box_reserve_flag(box_num_pre, 0); // -1 abondan, 1 save
    ApplyRotateNms(rotate_overlapped_blob_->cpu_data(),
        box_num_pre, res_sorted_indices, &box_reserve_flag, true);
    
    // // Push back the selected information.
    int box_num_post = 0;
    for (size_t i = 0; i < box_reserve_flag.size(); ++i) {
        if (box_reserve_flag[i] == 1) {
            ++box_num_post;
            if (box_num_post > nms_post_max_size_) {
                return nms_post_max_size_;
            }
            kept_indices->emplace_back(res_sorted_indices[i] + all_res_num);
        }
    }
    return box_num_post;
}

void CPDetection::DecodeValidObjects(std::vector<int>* kept_indices) {
    int box_range = head_map_["reg"] + head_map_["hei"] + head_map_["dim"];
    int dir_range = head_map_["rot"];
    std::vector<int> cls_range {0};

    float* box_data = output_box_blob_->mutable_gpu_data();
    float* cls_data = output_cls_blob_->mutable_gpu_data();
    float* dir_data = output_dir_blob_->mutable_gpu_data();

    // DEVICE_SAVE<float>(out_box_blob->gpu_data(), head_x_size_ * head_y_size_, 18, "05_bbox_preds.txt");
    // DEVICE_SAVE<float>(out_cls_blob->gpu_data(), head_x_size_ * head_y_size_, 4, "06_cls_scores.txt");
    // DEVICE_SAVE<float>(out_dir_blob->gpu_data(), head_x_size_ * head_y_size_, 6, "07_dir_scores.txt");
    float* all_res_box_data = all_res_box_blob_->mutable_gpu_data();
    float* all_res_conf_data = all_res_conf_blob_->mutable_gpu_data();
    int* all_res_cls_data = all_res_cls_blob_->mutable_gpu_data();
    BASE_GPU_CHECK(cudaMemsetAsync(all_res_box_data, 0.f, sizeof(float) * max_candidate_num_ * kBoxBlockSize, stream_));
    BASE_GPU_CHECK(cudaMemsetAsync(all_res_conf_data, 0.f, sizeof(float) * max_candidate_num_, stream_));
    BASE_GPU_CHECK(cudaMemsetAsync(all_res_cls_data, -1, sizeof(int) * max_candidate_num_, stream_));
  
    kept_indices->reserve(num_tasks_ * nms_post_max_size_);
    int all_res_box_num = 0;
    for (int i = 0; i < num_tasks_; ++i) {
        const float* reg = output_box_blob_->gpu_data() + output_box_blob_->offset(0, box_range * i);
        const float* hei = output_box_blob_->gpu_data() + output_box_blob_->offset(0, box_range * i + head_map_["reg"]);
        const float* dim = output_box_blob_->gpu_data() + output_box_blob_->offset(0, box_range * i + head_map_["reg"] + head_map_["hei"]);
        const float* cls = output_cls_blob_->gpu_data() + output_cls_blob_->offset(0, cls_range[i]);
        const float* rot = output_dir_blob_->gpu_data() + output_dir_blob_->offset(0, dir_range * i);
        cls_range.push_back(cls_range[i] + num_classes_in_task_[i]);

        int* res_box_num_data = res_box_num_blob_->mutable_gpu_data();
        float* res_box_data = res_box_blob_->mutable_gpu_data();
        float* res_conf_data = res_conf_blob_->mutable_gpu_data();
        int* res_cls_data = res_cls_blob_->mutable_gpu_data();
        float* score_thresh_per_class_data = score_class_map_blob_->mutable_gpu_data();

        BASE_GPU_CHECK(cudaMemsetAsync(res_box_num_data, 0, sizeof(int), stream_));
        BASE_GPU_CHECK(cudaMemsetAsync(res_box_data, 0.f, sizeof(float) * nms_pre_max_size_ * kBoxBlockSize, stream_));
        BASE_GPU_CHECK(cudaMemsetAsync(res_conf_data, 0.f, sizeof(float) * nms_pre_max_size_, stream_));
        BASE_GPU_CHECK(cudaMemsetAsync(res_cls_data, -1, sizeof(int) * nms_pre_max_size_, stream_));
        BASE_GPU_CHECK(cudaMemcpyAsync(score_thresh_per_class_data, &score_thresh_map_[0],
            sizeof(float) * num_classes_, cudaMemcpyHostToDevice, stream_));
        
        int block_size = DIVUP(head_map_size_, kGPUThreadSize);
        DecodeObjectKernel<<<block_size, kGPUThreadSize, 0, stream_>>>(head_map_size_,
            nms_pre_max_size_, x_min_range_, y_min_range_, voxel_x_size_, voxel_y_size_,
            head_x_size_, head_y_size_, downsample_size_,
            num_classes_in_task_[i], cls_range[i], score_thresh_per_class_data,
            reg, hei, dim, cls, rot, res_box_num_data, res_box_data, res_conf_data, res_cls_data);
        
        BASE_GPU_CHECK(cudaStreamSynchronize(stream_));
        const int* res_box_num = res_box_num_blob_->cpu_data();
        // const int* res_box_num = res_box_num_blob_->gpu_data();
        int box_num_pre = res_box_num[0] > nms_pre_max_size_ ? nms_pre_max_size_: res_box_num[0];
        if (box_num_pre <= 0) {
            continue;
        }
        
        // copy result
        block_size = DIVUP(box_num_pre, kGPUThreadSize);
        GetAllRemainBoxInfoKernel<<<block_size, kGPUThreadSize, 0, stream_>>>(box_num_pre,
            all_res_box_num, res_box_data, res_conf_data, res_cls_data,
            all_res_box_data, all_res_conf_data, all_res_cls_data);

        // nms
        int box_num_post = ApplyNmsGPU(box_num_pre, all_res_box_num, kept_indices);
        // int box_num_post = 0;
        // AERROR << "task " << i << " gets " << res_box_num[0] 
        //        << " objects before nms, and " << box_num_post << " after nms";
        all_res_box_num += box_num_pre;
    }
}

void CPDetection::GetObjectsAndAssignPointsGPU() {
    // get outputs
    Timer timer;
    std::vector<int> kept_indices;
    DecodeValidObjects(&kept_indices);
    double decode_time = timer.toc(true);
    
    // Get objects
    size_t box_num = kept_indices.size();
    if (box_num <= 0) {
        return;
    }

    // Assign points to box
    // AssignPoints2Boxid(kept_indices);
    SimpleAssignPoints2Boxid(kept_indices);
    BASE_GPU_CHECK(cudaStreamSynchronize(stream_));
    double assign_time = timer.toc(true);
    
    res_outputs_.clear();
    res_outputs_.resize(box_num);
    for (size_t i = 0; i < box_num; ++i) {
        const auto index = kept_indices[i];
        auto cand_bbox = all_res_box_blob_->cpu_data() + all_res_box_blob_->offset(index);
        auto cand_conf = all_res_conf_blob_->cpu_data() + index;
        auto cand_cls = all_res_cls_blob_->cpu_data() + index;
        float x = cand_bbox[0];
        float y = cand_bbox[1];
        float r = cand_bbox[6];
        if (enable_rotate_45degree_) {
            x =  0.707107 * cand_bbox[0] + 0.707107 * cand_bbox[1];
            y = -0.707107 * cand_bbox[0] + 0.707107 * cand_bbox[1];
            r -= M_PI / 4;
        }
        r += (r <= -M_PI ? M_PI : 0.0);
        r -= (r >= M_PI ? M_PI : 0.0);

        res_outputs_.at(i).clear();
        res_outputs_.at(i).x = x;
        res_outputs_.at(i).y = y;
        res_outputs_.at(i).z = cand_bbox[2] - cand_bbox[5] / 2;
        res_outputs_.at(i).l = cand_bbox[3];
        res_outputs_.at(i).w = cand_bbox[4];
        res_outputs_.at(i).h = cand_bbox[5];
        res_outputs_.at(i).yaw = r;
        res_outputs_.at(i).confidence = *cand_conf;
        res_outputs_.at(i).model_type_index = *cand_cls;
    }
    const int* valid_point_num = valid_point_num_blob_->cpu_data();
    const int* valid_point2boxid = valid_point2boxid_blob_->cpu_data();
    const int* valid_point_indices = valid_point_indices_blob_->cpu_data();
    ADEBUG << "valid_point_num: " << valid_point_num[0] << std::endl;
    if (valid_point_num[0] > max_valid_point_size_) {
        AERROR << "valid_point_num: " << valid_point_num[0] 
               << ", should > max_valid_point_size_: " << max_valid_point_size_;
        max_valid_point_size_ = std::max(max_valid_point_size_, total_cloud_size_);
    }
    for (int i = 0; i < valid_point_num[0]; ++i) {
        int point_idx = valid_point_indices[i];
        int box_idx = valid_point2boxid[i];
        if (box_idx < 0 || box_idx >= static_cast<int>(box_num)) { 
            continue; 
        }
        if (point_idx < 0 || point_idx >= total_cloud_size_) {
            continue;
        }
        bool semantic_flag = (static_cast<PointSemanticLabel>(
            original_cloud_->points_semantic_label(point_idx) & 15) ==
                PointSemanticLabel::GROUND);
        bool raw_flag = (original_cloud_->points_label(point_idx) ==
            static_cast<uint8_t>(LidarPointLabel::GROUND));
        if ((remove_semantic_ground_ && semantic_flag) ||
            (remove_raw_ground_ && raw_flag)) {
            continue;
        }
        res_outputs_.at(box_idx).AddPointId(point_idx);
    }
    double post_time = timer.toc(true);
    ADEBUG << "[CPDetPostProcess] decode_time: " << decode_time
           << ", assign_time: " << assign_time
           << ", post_time: " << post_time;
}

void CPDetection::GetObjectsGPU() {
    // get outputs
    Timer timer;
    std::vector<int> kept_indices;
    DecodeValidObjects(&kept_indices);
    double decode_time = timer.toc(true);

    // Get objects
    size_t box_num = kept_indices.size();
    int all_res_box_num = all_res_box_blob_->shape(0); 
    if (box_num <= 0) {
        return;
    }

    // get objects
    res_outputs_.clear();
    res_outputs_.resize(box_num);
    for (size_t i = 0; i < box_num; ++i) {
        const auto index = kept_indices[i];
        auto cand_bbox = all_res_box_blob_->cpu_data() + all_res_box_blob_->offset(index);
        auto cand_conf = all_res_conf_blob_->cpu_data() + index;
        auto cand_cls = all_res_cls_blob_->cpu_data() + index;
        float x = cand_bbox[0];
        float y = cand_bbox[1];
        float r = cand_bbox[6];
        if (enable_rotate_45degree_) {
            x =  0.707107 * cand_bbox[0] + 0.707107 * cand_bbox[1];
            y = -0.707107 * cand_bbox[0] + 0.707107 * cand_bbox[1];
            r -= M_PI / 4;
        }
        r += (r <= -M_PI ? M_PI : 0.0);
        r -= (r >= M_PI ? M_PI : 0.0);

        res_outputs_.at(i).clear();
        res_outputs_.at(i).x = x;
        res_outputs_.at(i).y = y;
        res_outputs_.at(i).z = cand_bbox[2] - cand_bbox[5] / 2;
        res_outputs_.at(i).l = cand_bbox[3];
        res_outputs_.at(i).w = cand_bbox[4];
        res_outputs_.at(i).h = cand_bbox[5];
        res_outputs_.at(i).yaw = r;
        res_outputs_.at(i).confidence = *cand_conf;
        res_outputs_.at(i).model_type_index = *cand_cls;
    }
    // AERROR << "total pointcloud_num: " << original_cloud_->size()
    //        << "valid_point_num: " << valid_point.size() << std::endl;
    // if (valid_point.size() > max_valid_point_size_) {
    //     AERROR << "valid_point_num: " << valid_point.size() 
    //            << ", should > max_valid_point_size_: " << max_valid_point_size_;
    //     max_valid_point_size_ = std::max(max_valid_point_size_, total_cloud_size_);
    // }
    // for (int i = 0; i < valid_point.size(); ++i) {
    //     int point_idx = count2point[i];
    //     int box_idx = count2box[i];
    //     if (box_idx < 0 || box_idx >= static_cast<int>(box_num)) { 
    //         continue; 
    //     }
    //     bool semantic_flag = (static_cast<PointSemanticLabel>(
    //         original_cloud_->points_semantic_label(point_idx) & 15) ==
    //             PointSemanticLabel::GROUND);
    //     bool raw_flag = (original_cloud_->points_label(point_idx) ==
    //         static_cast<uint8_t>(LidarPointLabel::GROUND));
    //     if ((remove_semantic_ground_ && semantic_flag) ||
    //         (remove_raw_ground_ && raw_flag)) {
    //         continue;
    //     }
    //     res_outputs_.at(box_idx).AddPointId(point_idx);
    // }
    double post_time = timer.toc(true);
    ADEBUG << "[CPDetPostProcess] decode_time: " << decode_time << ", post_time: " << post_time;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo