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

#include "region_output.h"
#include "thrust/functional.h"
#include "thrust/sort.h"
#include "boost/iterator/counting_iterator.hpp"

namespace apollo {
namespace perception {
__host__ __device__ float bbox_size_gpu(const float *bbox,
                    const bool normalized) {
    if (bbox[2] < bbox[0] || bbox[3] < bbox[1]) {
        // If bbox is invalid (e.g. xmax < xmin or ymax < ymin), return 0.
        return float(0.);
    } else {
        const float width = bbox[2] - bbox[0];
        const float height = bbox[3] - bbox[1];
        if (normalized) {
            return width * height;
        } else {
            // If bbox is not within range [0, 1].
            return (width + 1) * (height + 1);
        }
    }
}
__host__ __device__ float jaccard_overlap_gpu(const float *bbox1,
                          const float *bbox2) {
    if (bbox2[0] > bbox1[2] || bbox2[2] < bbox1[0] ||
        bbox2[1] > bbox1[3] || bbox2[3] < bbox1[1]) {
        return float(0.);
    } else {
        const float inter_xmin = max(bbox1[0], bbox2[0]);
        const float inter_ymin = max(bbox1[1], bbox2[1]);
        const float inter_xmax = min(bbox1[2], bbox2[2]);
        const float inter_ymax = min(bbox1[3], bbox2[3]);

        const float inter_width = inter_xmax - inter_xmin;
        const float inter_height = inter_ymax - inter_ymin;
        const float inter_size = inter_width * inter_height;

        const float bbox1_size = bbox_size_gpu(bbox1, true);
        const float bbox2_size = bbox_size_gpu(bbox2, true);

        return inter_size / (bbox1_size + bbox2_size - inter_size);
    }
}

__global__ void compute_overlapped_by_idx_kernel(const int nthreads,
                                                 const float *bbox_data,
                                                 const float overlap_threshold,
                                                 const int *idx, const int num_idx,
                                                 bool *overlapped_data) {
    for (int index = blockIdx.x * blockDim.x + threadIdx.x;
         index < (nthreads); index += blockDim.x * gridDim.x) {
        const int j = index % num_idx;
        const int i = (index / num_idx);
        if (i == j) {
            // Ignore same bbox.
            return;
        }
        // Compute overlap between i-th bbox and j-th bbox.
        const int start_loc_i = idx[i] * s_box_block_size;
        const int start_loc_j = idx[j] * s_box_block_size;
        const float overlap = jaccard_overlap_gpu(bbox_data + start_loc_i,
                                                  bbox_data + start_loc_j);
        if (overlap > overlap_threshold) {
            overlapped_data[index] = true;
        } else {
            //const float *b1 = bbox_data + start_loc_i;
            //const float *b2 = bbox_data + start_loc_j;

            overlapped_data[index] = false;
        }
    }
}
void compute_overlapped_by_idx_gpu(const int nthreads,
                                   const float *bbox_data, const float overlap_threshold,
                                   const int *idx, const int num_idx, bool *overlapped_data) {
    // NOLINT_NEXT_LINE(whitespace/operators)
    const int thread_size = 512;
    int block_size = (nthreads + thread_size - 1) / thread_size;
    compute_overlapped_by_idx_kernel << < block_size, thread_size >> >
                                                      (nthreads, bbox_data, overlap_threshold,
                                                              idx, num_idx, overlapped_data);
}
void apply_nms_gpu(const float *bbox_data, const float *conf_data,
                   const int num_bboxes, const float confidence_threshold,
                   const int top_k, const float nms_threshold, std::vector<int> *indices,
                   std::shared_ptr <caffe::SyncedMemory> overlapped, std::shared_ptr <caffe::SyncedMemory> idx_sm) {
    // Keep part of detections whose scores are higher than confidence threshold.
    cudaDeviceSynchronize();
    std::vector<int> idx;
    std::vector<float> confidences;
    for (int i = 0; i < num_bboxes; ++i) {
        if (conf_data[i] > confidence_threshold) {
            idx.push_back(i);
            confidences.push_back(conf_data[i]);
        }
    }
    int num_remain = confidences.size();
    if (num_remain == 0) {
        return;
    }
    // Sort detections based on score.
    thrust::sort_by_key(&confidences[0], &confidences[0] + num_remain, &idx[0],
                        thrust::greater<float>());
    if (top_k > -1 && top_k < num_remain) {
        num_remain = top_k;
    }

    int *idx_data = (int *) idx_sm->mutable_cpu_data();
    std::copy(idx.begin(), idx.begin() + num_remain, idx_data);

    bool *overlapped_data = (bool *) overlapped->mutable_gpu_data();
    int total_bboxes = num_remain * num_remain;
    compute_overlapped_by_idx_gpu(total_bboxes, bbox_data, nms_threshold,
                                  (const int *) idx_sm->gpu_data(), num_remain, overlapped_data);
    cudaDeviceSynchronize();

    // Do non-maximum suppression based on overlapped results.
    const bool *overlapped_results = (const bool *) overlapped->cpu_data();

    std::vector<int> selected_indices;
    apply_nms(overlapped_results, num_remain, &selected_indices);

    // Put back the selected information.
    for (int i = 0; i < (int)selected_indices.size(); ++i) {
        indices->push_back(idx[selected_indices[i]]);
    }
}
void apply_nms(const bool *overlapped, const int num, std::vector<int> *indices) {
    std::vector<int> index_vec(boost::counting_iterator<int>(0),
                               boost::counting_iterator<int>(num));
    // Do nms.
    indices->clear();
    while (index_vec.size() != 0) {
        // Get the current highest score box.
        int best_idx = index_vec.front();
        indices->push_back(best_idx);
        // Erase the best box.
        index_vec.erase(index_vec.begin());

        for (std::vector<int>::iterator it = index_vec.begin(); it != index_vec.end();) {
            int cur_idx = *it;

            // Remove it if necessary
            if (overlapped[best_idx * num + cur_idx]) {
                it = index_vec.erase(it);
            } else {
                ++it;
            }
        }
    }
}
}
}
