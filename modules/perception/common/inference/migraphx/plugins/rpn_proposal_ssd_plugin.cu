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

#include <thrust/sort.h>
#include <thrust/sequence.h>

#include "modules/perception/common/inference/migraphx/plugins/kernels.h"
#include "modules/perception/common/inference/migraphx/plugins/rpn_proposal_ssd_plugin.h"

namespace apollo {
namespace perception {
namespace inference {

// TODO(chenjiahao): add heat_map_b as anchor_offset
// output anchors dims: [H, W, num_anchor_per_point, 4]
__global__ void generate_anchors_kernel(const int height, const int width,
                                        const float anchor_stride,
                                        const int num_anchor_per_point,
                                        const float *anchor_heights,
                                        const float *anchor_widths,
                                        float *anchors) {
  int index = threadIdx.x + blockIdx.x * blockDim.x;
  const int num_anchor = height * width * num_anchor_per_point;
  if (index >= num_anchor) {
    return;
  }

  float anchor_offset = 0;
  int pos_index = index / num_anchor_per_point;
  int anchor_id = index % num_anchor_per_point;
  int w_i = pos_index % width;
  int h_i = pos_index / width;

  // center coordinates
  float x_ctr = w_i * anchor_stride + anchor_offset;
  float y_ctr = h_i * anchor_stride + anchor_offset;

  float x_min = x_ctr - 0.5 * (anchor_widths[anchor_id] - 1);
  float y_min = y_ctr - 0.5 * (anchor_heights[anchor_id] - 1);
  float x_max = x_ctr + 0.5 * (anchor_widths[anchor_id] - 1);
  float y_max = y_ctr + 0.5 * (anchor_heights[anchor_id] - 1);

  anchors[index * 4] = x_min;
  anchors[index * 4 + 1] = y_min;
  anchors[index * 4 + 2] = x_max;
  anchors[index * 4 + 3] = y_max;
}

// in_boxes dims: [N, num_box_per_point * 4, H, W],
// out_boxes dims: [N, H * W * num_box_per_point， 4]
template <typename Dtype>
__global__ void reshape_boxes_kernel(const int nthreads, const Dtype *in_boxes,
                                     const int height, const int width,
                                     const int num_box_per_point,
                                     Dtype *out_boxes) {
  int index = threadIdx.x + blockIdx.x * blockDim.x;
  if (index < nthreads) {
    int num_point = height * width;

    int batch_id = index / num_point / num_box_per_point / 4;
    int feature_id = index % 4;
    int box_id = (index / 4) % num_box_per_point;
    int point_id = (index / num_box_per_point / 4) % num_point;

    int in_index =
        ((batch_id * num_box_per_point + box_id) * 4 + feature_id) * num_point +
        point_id;
    out_boxes[index] = in_boxes[in_index];
  }
}

// in_scores dims: [N, 2 * num_box_per_point, H, W],
// out_scores dims: [N, H * W * num_box_per_point, 2]
template <typename Dtype>
__global__ void reshape_scores_kernel(const int nthreads,
                                      const Dtype *in_scores, const int height,
                                      const int width,
                                      const int num_box_per_point,
                                      Dtype *out_scores) {
  int index = threadIdx.x + blockIdx.x * blockDim.x;
  if (index < nthreads) {
    int num_point = height * width;

    int batch_id = index / num_point / num_box_per_point / 2;
    int class_id = index % 2;
    int box_id = (index / 2) % num_box_per_point;
    int point_id = (index / num_box_per_point / 2) % num_point;

    int in_index =
        ((batch_id * 2 + class_id) * num_box_per_point + box_id) * num_point +
        point_id;
    out_scores[index] = in_scores[in_index];
  }
}

int RPNProposalSSDPlugin::enqueue(int batchSize, const void *const *inputs,
                                  void **outputs, void *workspace,
                                  cudaStream_t stream) {
  // dimsNCHW: [N, 2 * num_anchor_per_point, H, W]
  const float *rpn_cls_prob_reshape =
      reinterpret_cast<const float *>(inputs[0]);
  // dimsNCHW: [N, num_anchor_per_point * 4, H, W]
  const float *rpn_bbox_pred = reinterpret_cast<const float *>(inputs[1]);
  // dims: [N, 6, 1, 1]
  const float *im_info = reinterpret_cast<const float *>(inputs[2]);
  float *out_rois = reinterpret_cast<float *>(outputs[0]);

  float *host_im_info = new float[batchSize * 6]();
  BASE_GPU_CHECK(cudaMemcpyAsync(host_im_info, im_info,
                                  batchSize * 6 * sizeof(float),
                                  cudaMemcpyDeviceToHost, stream));

  const int origin_height = (int)(host_im_info[0]);
  const int origin_width = (int)(host_im_info[1]);
  int num_anchor = height_ * width_ * num_anchor_per_point_;
  int rpn_bbox_pred_size = batchSize * num_anchor * 4;
  int scores_size = batchSize * num_anchor * 2;
  int anchors_size = num_anchor * 4;
  int out_rois_size = batchSize * top_n_ * 5;

  // Using thrust::fill might cause crash
  float *init_out_rois = new float[out_rois_size]();
  std::fill_n(init_out_rois, out_rois_size, -1.0f);
  BASE_GPU_CHECK(cudaMemcpyAsync(out_rois, init_out_rois,
                                  out_rois_size * sizeof(float),
                                  cudaMemcpyHostToDevice, stream));

  int block_size, nthreads;

  // reshape to [N, num_anchor, 4]
  float *temp_rpn_bbox_pred;
  BASE_GPU_CHECK(cudaMalloc(reinterpret_cast<void **>(&temp_rpn_bbox_pred),
                             rpn_bbox_pred_size * sizeof(float)));
  nthreads = rpn_bbox_pred_size;
  block_size = (nthreads - 1) / thread_size_ + 1;
  reshape_boxes_kernel<<<block_size, thread_size_, 0, stream>>>(
      nthreads, rpn_bbox_pred, height_, width_, num_anchor_per_point_,
      temp_rpn_bbox_pred);

  // Normalization
  float *dev_bbox_mean, *dev_bbox_std;
  BASE_GPU_CHECK(
      cudaMalloc(reinterpret_cast<void **>(&dev_bbox_mean), 4 * sizeof(float)));
  BASE_GPU_CHECK(
      cudaMalloc(reinterpret_cast<void **>(&dev_bbox_std), 4 * sizeof(float)));
  BASE_GPU_CHECK(cudaMemcpyAsync(dev_bbox_mean, bbox_mean_, 4 * sizeof(float),
                                  cudaMemcpyHostToDevice, stream));
  BASE_GPU_CHECK(cudaMemcpyAsync(dev_bbox_std, bbox_std_, 4 * sizeof(float),
                                  cudaMemcpyHostToDevice, stream));
  repeatedly_mul_cuda(block_size, thread_size_, 0, stream, nthreads,
                      temp_rpn_bbox_pred, temp_rpn_bbox_pred, dev_bbox_std, 4);
  repeatedly_add_cuda(block_size, thread_size_, 0, stream, nthreads,
                      temp_rpn_bbox_pred, temp_rpn_bbox_pred, dev_bbox_mean, 4);

  // generate anchors
  float *anchors, *dev_anchor_heights, *dev_anchor_widths;
  BASE_GPU_CHECK(cudaMalloc(reinterpret_cast<void **>(&anchors),
                             anchors_size * sizeof(float)));
  BASE_GPU_CHECK(
      cudaMemsetAsync(anchors, 0, anchors_size * sizeof(float), stream));
  BASE_GPU_CHECK(cudaMalloc(reinterpret_cast<void **>(&dev_anchor_heights),
                             num_anchor_per_point_ * sizeof(float)));
  BASE_GPU_CHECK(cudaMalloc(reinterpret_cast<void **>(&dev_anchor_widths),
                             num_anchor_per_point_ * sizeof(float)));
  BASE_GPU_CHECK(cudaMemsetAsync(
      dev_anchor_heights, 0, num_anchor_per_point_ * sizeof(float), stream));
  BASE_GPU_CHECK(cudaMemsetAsync(
      dev_anchor_widths, 0, num_anchor_per_point_ * sizeof(float), stream));
  BASE_GPU_CHECK(cudaMemcpyAsync(dev_anchor_heights, anchor_heights_,
                                  num_anchor_per_point_ * sizeof(float),
                                  cudaMemcpyHostToDevice, stream));
  BASE_GPU_CHECK(cudaMemcpyAsync(dev_anchor_widths, anchor_widths_,
                                  num_anchor_per_point_ * sizeof(float),
                                  cudaMemcpyHostToDevice, stream));
  block_size = (anchors_size - 1) / thread_size_ + 1;
  generate_anchors_kernel<<<block_size, thread_size_, 0, stream>>>(
      height_, width_, heat_map_a_, num_anchor_per_point_, dev_anchor_heights,
      dev_anchor_widths, anchors);

  // decode bbox
  float *proposals;
  BASE_GPU_CHECK(cudaMalloc(reinterpret_cast<void **>(&proposals),
                             rpn_bbox_pred_size * sizeof(float)));
  BASE_GPU_CHECK(cudaMemsetAsync(proposals, 0,
                                  rpn_bbox_pred_size * sizeof(float), stream));
  nthreads = batchSize * num_anchor;
  block_size = (nthreads - 1) / thread_size_ + 1;
  bbox_transform_inv_cuda(block_size, thread_size_, 0, stream, nthreads,
                          anchors, temp_rpn_bbox_pred, num_anchor, 1,
                          proposals);

  // clip boxes, i.e. refine proposals which are out of map
  if (refine_out_of_map_bbox_) {
    nthreads = rpn_bbox_pred_size;
    block_size = (nthreads - 1) / thread_size_ + 1;
    clip_boxes_cuda(block_size, thread_size_, 0, stream, nthreads, proposals,
                    (float)origin_height, (float)origin_width);
  }

  // reshape scores to [N, num_anchor, 2]
  float *temp_scores;
  BASE_GPU_CHECK(cudaMalloc(reinterpret_cast<void **>(&temp_scores),
                             scores_size * sizeof(float)));
  nthreads = scores_size;
  block_size = (nthreads - 1) / thread_size_ + 1;
  reshape_scores_kernel<<<block_size, thread_size_, 0, stream>>>(
      nthreads, rpn_cls_prob_reshape, height_, width_, num_anchor_per_point_,
      temp_scores);

  // filter boxes according to min_size_mode and threshold_objectness
  float *filtered_proposals, *filtered_scores;
  int *filtered_count;
  BASE_GPU_CHECK(cudaMalloc(reinterpret_cast<void **>(&filtered_proposals),
                             rpn_bbox_pred_size * sizeof(float)));
  BASE_GPU_CHECK(cudaMalloc(reinterpret_cast<void **>(&filtered_scores),
                             batchSize * num_anchor * sizeof(float)));
  BASE_GPU_CHECK(cudaMalloc(reinterpret_cast<void **>(&filtered_count),
                             batchSize * sizeof(int)));
  BASE_GPU_CHECK(cudaMemsetAsync(filtered_proposals, 0,
                                  rpn_bbox_pred_size * sizeof(float), stream));
  BASE_GPU_CHECK(cudaMemsetAsync(
      filtered_scores, 0, batchSize * num_anchor * sizeof(float), stream));
  BASE_GPU_CHECK(
      cudaMemsetAsync(filtered_count, 0, batchSize * sizeof(int), stream));
  nthreads = batchSize * num_anchor;
  block_size = (nthreads - 1) / thread_size_ + 1;
  // TODO(chenjiahao): filter area
  filter_boxes_cuda(block_size, thread_size_, 0, stream, nthreads, proposals,
                    temp_scores, nullptr, num_anchor, 1, 2, 0, 0, 1,
                    min_size_mode_, min_size_h_, min_size_w_,
                    threshold_objectness_, filtered_proposals, filtered_scores,
                    nullptr, filtered_count);

  int *host_filtered_count = new int[batchSize]();
  BASE_GPU_CHECK(cudaMemcpyAsync(host_filtered_count, filtered_count,
                                  batchSize * sizeof(int),
                                  cudaMemcpyDeviceToHost, stream));

  // descending sort proposals by score
  int *sorted_indexes;
  BASE_GPU_CHECK(cudaMalloc(reinterpret_cast<void **>(&sorted_indexes),
                             batchSize * num_anchor * sizeof(int)));
  for (int i = 0; i < batchSize; ++i) {
    thrust::sequence(thrust::device, sorted_indexes + i * num_anchor,
                     sorted_indexes + i * num_anchor + host_filtered_count[i]);
    thrust::sort_by_key(
        thrust::device, filtered_scores + size_t(i * num_anchor),
        filtered_scores + size_t(i * num_anchor + host_filtered_count[i]),
        sorted_indexes + i * num_anchor, thrust::greater<float>());
  }

  // keep max N candidates
  float *pre_nms_proposals;
  BASE_GPU_CHECK(cudaMalloc(reinterpret_cast<void **>(&pre_nms_proposals),
                             batchSize * max_candidate_n_ * 4 * sizeof(float)));
  BASE_GPU_CHECK(cudaMemsetAsync(
      pre_nms_proposals, 0, batchSize * max_candidate_n_ * 4 * sizeof(float),
      stream));
  nthreads = batchSize * max_candidate_n_;
  block_size = (nthreads - 1) / thread_size_ + 1;
  keep_topN_boxes_cuda(block_size, thread_size_, 0, stream, nthreads,
                       filtered_proposals, nullptr, nullptr, sorted_indexes,
                       filtered_count, false, num_anchor, 0, max_candidate_n_,
                       pre_nms_proposals, nullptr, nullptr);

  // Nms, keep top N proposals and output final proposals
  // output dims: [num_roi, 5] (axis-1: batch_id, x_min, y_min, x_max, y_max)
  int acc_box_num = 0;
  for (int i = 0; i < batchSize; ++i) {
    int cur_filter_count = std::min(host_filtered_count[i], max_candidate_n_);
    NmsForward(
        false, cur_filter_count, 4, overlap_ratio_, max_candidate_n_, top_n_, i,
        0, pre_nms_proposals + size_t(i * max_candidate_n_ * 4), nullptr,
        nullptr, out_rois + size_t(acc_box_num * 5), &acc_box_num, stream);
  }

  out_rois_num_ = acc_box_num;

  // Free cuda memory
  BASE_GPU_CHECK(cudaFree(temp_rpn_bbox_pred));
  BASE_GPU_CHECK(cudaFree(dev_bbox_mean));
  BASE_GPU_CHECK(cudaFree(dev_bbox_std));
  BASE_GPU_CHECK(cudaFree(anchors));
  BASE_GPU_CHECK(cudaFree(dev_anchor_heights));
  BASE_GPU_CHECK(cudaFree(dev_anchor_widths));
  BASE_GPU_CHECK(cudaFree(proposals));
  BASE_GPU_CHECK(cudaFree(temp_scores));
  BASE_GPU_CHECK(cudaFree(filtered_proposals));
  BASE_GPU_CHECK(cudaFree(filtered_scores));
  BASE_GPU_CHECK(cudaFree(filtered_count));
  BASE_GPU_CHECK(cudaFree(sorted_indexes));
  BASE_GPU_CHECK(cudaFree(pre_nms_proposals));

  // Free host memory
  delete[] host_im_info;
  delete[] host_filtered_count;
  delete[] init_out_rois;

  return 0;
}
}  // namespace inference
}  // namespace perception
}  // namespace apollo
