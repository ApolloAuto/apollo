/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/base/common.h"

namespace apollo {
namespace perception {
namespace inference {

#define NUM_2D_BOX_CORNERS_MACRO 4
#define NUM_THREADS_MACRO 64

#define CUDA_KERNEL_LOOP(i, n)                                 \
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < (n); \
       i += blockDim.x * gridDim.x)

#define DIVUP(m, n) ((m) / (n) + ((m) % (n) > 0))

void bbox_transform_inv_cuda(int block_size, int thread_size, int shared_mem,
                             cudaStream_t stream, const int nthreads,
                             const float *boxes, const float *deltas,
                             const int num_box, const int num_channel,
                             float *out_boxes);

void clip_boxes_cuda(int block_size, int thread_size, int shared_mem,
                     cudaStream_t stream, const int nthreads, float *boxes,
                     const float height, const float width);

void filter_boxes_cuda(
    int block_size, int thread_size, int shared_mem, cudaStream_t stream,
    const int nthreads, const float *boxes, const float *scores,
    const float *all_probs, const int num_box, const int num_channel,
    const int num_class, const int num_prob, const int filter_channel,
    const int filter_class, const int min_size_mode, const float min_size_h,
    const float min_size_w, const float threshold_score, float *filtered_boxes,
    float *filtered_scores, float *filtered_all_probs, int *filtered_count);

void keep_topN_boxes_cuda(int block_size, int thread_size, int shared_mem,
                          cudaStream_t stream, const int nthreads,
                          const float *boxes, const float *scores,
                          const float *all_probs, const int *indexes,
                          const int *count, const bool keep_score,
                          const int num_box, const int num_prob, const int topN,
                          float *out_boxes, float *out_scores,
                          float *out_all_probs);

void repeatedly_add_cuda(int block_size, int thread_size, int shared_mem,
                         cudaStream_t stream, const int nthreads,
                         const float *in_data, float *out_data,
                         const float *add_vec, int add_vec_size);

void repeatedly_mul_cuda(int block_size, int thread_size, int shared_mem,
                         cudaStream_t stream, const int nthreads,
                         const float *in_data, float *out_data,
                         const float *mul_vec, int mul_vec_size);

void slice2d_cuda(int block_size, int thread_size, int shared_mem,
                  cudaStream_t stream, const int nthreads, const float *in_data,
                  float *out_data, const int *slice_axises, int slice_axis_num,
                  int input_axis_size);

/**
 * @brief GPU Non-Maximum Suppresion for network output
 * @param[in] rpn_proposal_output_score Whether to output scores
 * @param[in] host_filter_count Number of filtered output
 * @param[in] num_box_corners Number of corners for 2D box
 * @param[in] nms_overlap_threshold IOU threshold for NMS
 * @param[in] num_candidate Pre-defined maximum number of candidates
 * @param[in] top_n Pre-defined maximum number of output boxes
 * @param[in] batch_id Id of current batch instance
 * @param[in] num_prob Number of probs
 * @param[in] dev_sorted_box_for_nms Bounding box output sorted by score
 * @param[in] scores Scores of boxes
 * @param[in] all_probs Probs of boxes for all classes and objectness
 * @param[in] out_boxes Output boxes
 * @param[in] acc_box_num Accumulated box num
 * @details NMS in GPU and postprocessing for selecting box in CPU
 */
void NmsForward(bool rpn_proposal_output_score, int host_filter_count,
                int num_box_corners, float nms_overlap_threshold,
                int num_candidate, int top_n, int batch_id, int num_prob,
                float *dev_sorted_box_for_nms, float *scores, float *all_probs,
                float *out_boxes, int *acc_box_num, cudaStream_t stream);

}  // namespace inference
}  // namespace perception
}  // namespace apollo
