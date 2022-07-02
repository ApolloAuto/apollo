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

/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// headers in CUDA
#include <thrust/device_ptr.h>
#include <thrust/host_vector.h>
#include <thrust/sort.h>
#include <thrust/sequence.h>

// headers in local files
#include "modules/perception/lidar/lib/detector/point_pillars_detection/postprocess_cuda.h"

namespace apollo {
namespace perception {
namespace lidar {

__global__ void filter_kernel(
    const float* box_preds, const float* cls_preds, const float* dir_preds,
    const int* anchor_mask, const float* dev_anchors_px,
    const float* dev_anchors_py, const float* dev_anchors_pz,
    const float* dev_anchors_dx, const float* dev_anchors_dy,
    const float* dev_anchors_dz, const float* dev_anchors_ro,
    float* filtered_box, float* filtered_score, int* filtered_label,
    int* filtered_dir, float* box_for_nms, int* filter_count,
    const float float_min, const float float_max, const float score_threshold,
    const int num_box_corners, const int num_output_box_feature,
    const int num_class) {
  // boxes ([N, 7] Tensor): normal boxes: x, y, z, w, l, h, r
  int tid = threadIdx.x + blockIdx.x * blockDim.x;
  // sigmoid function
  float top_score = 0;
  int top_label = 0;
  for (int i = 0; i < num_class; ++i) {
    float score = 1 / (1 + expf(-cls_preds[tid * num_class + i]));
    if (score > top_score) {
      top_score = score;
      top_label = i;
    }
  }
  if (anchor_mask[tid] == 1 && top_score > score_threshold) {
    int counter = atomicAdd(filter_count, 1);
    float za = dev_anchors_pz[tid] + dev_anchors_dz[tid] / 2;

    // decode network output
    float diagonal = sqrtf(dev_anchors_dx[tid] * dev_anchors_dx[tid] +
                           dev_anchors_dy[tid] * dev_anchors_dy[tid]);
    float box_px = box_preds[tid * num_output_box_feature + 0] * diagonal +
                   dev_anchors_px[tid];
    float box_py = box_preds[tid * num_output_box_feature + 1] * diagonal +
                   dev_anchors_py[tid];
    float box_pz =
        box_preds[tid * num_output_box_feature + 2] * dev_anchors_dz[tid] + za;
    float box_dx =
        expf(box_preds[tid * num_output_box_feature + 3]) * dev_anchors_dx[tid];
    float box_dy =
        expf(box_preds[tid * num_output_box_feature + 4]) * dev_anchors_dy[tid];
    float box_dz =
        expf(box_preds[tid * num_output_box_feature + 5]) * dev_anchors_dz[tid];
    float box_ro =
        box_preds[tid * num_output_box_feature + 6] + dev_anchors_ro[tid];

    box_pz = box_pz - box_dz / 2;

    filtered_box[counter * num_output_box_feature + 0] = box_px;
    filtered_box[counter * num_output_box_feature + 1] = box_py;
    filtered_box[counter * num_output_box_feature + 2] = box_pz;
    filtered_box[counter * num_output_box_feature + 3] = box_dx;
    filtered_box[counter * num_output_box_feature + 4] = box_dy;
    filtered_box[counter * num_output_box_feature + 5] = box_dz;
    filtered_box[counter * num_output_box_feature + 6] = box_ro;
    filtered_score[counter] = top_score;
    filtered_label[counter] = top_label;

    int direction_label;
    if (dir_preds[tid * 2 + 0] < dir_preds[tid * 2 + 1]) {
      direction_label = 1;
    } else {
      direction_label = 0;
    }
    filtered_dir[counter] = direction_label;

    // convrt normal box(normal boxes: x, y, z, w, l, h, r) to box(xmin, ymin,
    // xmax, ymax) for nms calculation First: dx, dy -> box(x0y0, x0y1, x1y0,
    // x1y1)
    float corners[NUM_3D_BOX_CORNERS_MACRO] = {
        static_cast<float>(-0.5 * box_dx), static_cast<float>(-0.5 * box_dy),
        static_cast<float>(-0.5 * box_dx), static_cast<float>(0.5 * box_dy),
        static_cast<float>(0.5 * box_dx),  static_cast<float>(0.5 * box_dy),
        static_cast<float>(0.5 * box_dx),  static_cast<float>(-0.5 * box_dy)};

    // Second: Rotate, Offset and convert to point(xmin. ymin, xmax, ymax)
    float rotated_corners[NUM_3D_BOX_CORNERS_MACRO];
    float offset_corners[NUM_3D_BOX_CORNERS_MACRO];
    float sin_yaw = sinf(box_ro);
    float cos_yaw = cosf(box_ro);
    float xmin = float_max;
    float ymin = float_max;
    float xmax = float_min;
    float ymax = float_min;
    for (size_t i = 0; i < num_box_corners; ++i) {
      rotated_corners[i * 2 + 0] =
          cos_yaw * corners[i * 2 + 0] - sin_yaw * corners[i * 2 + 1];
      rotated_corners[i * 2 + 1] =
          sin_yaw * corners[i * 2 + 0] + cos_yaw * corners[i * 2 + 1];

      offset_corners[i * 2 + 0] = rotated_corners[i * 2 + 0] + box_px;
      offset_corners[i * 2 + 1] = rotated_corners[i * 2 + 1] + box_py;

      xmin = fminf(xmin, offset_corners[i * 2 + 0]);
      ymin = fminf(ymin, offset_corners[i * 2 + 1]);
      xmax = fmaxf(xmax, offset_corners[i * 2 + 0]);
      ymax = fmaxf(ymax, offset_corners[i * 2 + 1]);
    }
    // box_for_nms(num_box, 4)
    box_for_nms[counter * num_box_corners + 0] = xmin;
    box_for_nms[counter * num_box_corners + 1] = ymin;
    box_for_nms[counter * num_box_corners + 2] = xmax;
    box_for_nms[counter * num_box_corners + 3] = ymax;
  }
}

__global__ void sort_boxes_by_indexes_kernel(
    float* filtered_box, int* filtered_label, int* filtered_dir,
    float* box_for_nms, int* indexes, int filter_count,
    float* sorted_filtered_boxes, int* sorted_filtered_label,
    int* sorted_filtered_dir, float* sorted_box_for_nms,
    const int num_box_corners, const int num_output_box_feature) {
  int tid = threadIdx.x + blockIdx.x * blockDim.x;
  if (tid < filter_count) {
    int sort_index = indexes[tid];
    sorted_filtered_boxes[tid * num_output_box_feature + 0] =
        filtered_box[sort_index * num_output_box_feature + 0];
    sorted_filtered_boxes[tid * num_output_box_feature + 1] =
        filtered_box[sort_index * num_output_box_feature + 1];
    sorted_filtered_boxes[tid * num_output_box_feature + 2] =
        filtered_box[sort_index * num_output_box_feature + 2];
    sorted_filtered_boxes[tid * num_output_box_feature + 3] =
        filtered_box[sort_index * num_output_box_feature + 3];
    sorted_filtered_boxes[tid * num_output_box_feature + 4] =
        filtered_box[sort_index * num_output_box_feature + 4];
    sorted_filtered_boxes[tid * num_output_box_feature + 5] =
        filtered_box[sort_index * num_output_box_feature + 5];
    sorted_filtered_boxes[tid * num_output_box_feature + 6] =
        filtered_box[sort_index * num_output_box_feature + 6];

    sorted_filtered_label[tid] = filtered_label[sort_index];

    sorted_filtered_dir[tid] = filtered_dir[sort_index];

    sorted_box_for_nms[tid * num_box_corners + 0] =
        box_for_nms[sort_index * num_box_corners + 0];
    sorted_box_for_nms[tid * num_box_corners + 1] =
        box_for_nms[sort_index * num_box_corners + 1];
    sorted_box_for_nms[tid * num_box_corners + 2] =
        box_for_nms[sort_index * num_box_corners + 2];
    sorted_box_for_nms[tid * num_box_corners + 3] =
        box_for_nms[sort_index * num_box_corners + 3];
  }
}

PostprocessCuda::PostprocessCuda(const float float_min, const float float_max,
                                 const int num_anchor, const int num_class,
                                 const float score_threshold,
                                 const int num_threads,
                                 const float nms_overlap_threshold,
                                 const int num_box_corners,
                                 const int num_output_box_feature)
    : float_min_(float_min),
      float_max_(float_max),
      num_anchor_(num_anchor),
      num_class_(num_class),
      score_threshold_(score_threshold),
      num_threads_(num_threads),
      nms_overlap_threshold_(nms_overlap_threshold),
      num_box_corners_(num_box_corners),
      num_output_box_feature_(num_output_box_feature) {
  nms_cuda_ptr_.reset(
      new NmsCuda(num_threads, num_box_corners, nms_overlap_threshold));
}

void PostprocessCuda::DoPostprocessCuda(
    const float* rpn_box_output, const float* rpn_cls_output,
    const float* rpn_dir_output, int* dev_anchor_mask,
    const float* dev_anchors_px, const float* dev_anchors_py,
    const float* dev_anchors_pz, const float* dev_anchors_dx,
    const float* dev_anchors_dy, const float* dev_anchors_dz,
    const float* dev_anchors_ro, float* dev_filtered_box,
    float* dev_filtered_score, int* dev_filtered_label, int* dev_filtered_dir,
    float* dev_box_for_nms, int* dev_filter_count,
    std::vector<float>* out_detection, std::vector<int>* out_label) {
  const int num_blocks_filter_kernel = DIVUP(num_anchor_, num_threads_);
  filter_kernel<<<num_blocks_filter_kernel, num_threads_>>>(
      rpn_box_output, rpn_cls_output, rpn_dir_output, dev_anchor_mask,
      dev_anchors_px, dev_anchors_py, dev_anchors_pz, dev_anchors_dx,
      dev_anchors_dy, dev_anchors_dz, dev_anchors_ro, dev_filtered_box,
      dev_filtered_score, dev_filtered_label, dev_filtered_dir, dev_box_for_nms,
      dev_filter_count, float_min_, float_max_, score_threshold_,
      num_box_corners_, num_output_box_feature_, num_class_);

  int host_filter_count[1] = {0};
  GPU_CHECK(cudaMemcpy(host_filter_count, dev_filter_count, sizeof(int),
                       cudaMemcpyDeviceToHost));
  if (host_filter_count[0] == 0) {
    return;
  }

  int* dev_indexes;
  float *dev_sorted_filtered_box, *dev_sorted_box_for_nms;
  int *dev_sorted_filtered_label, *dev_sorted_filtered_dir;
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_indexes),
                       host_filter_count[0] * sizeof(int)));
  GPU_CHECK(cudaMalloc(
      reinterpret_cast<void**>(&dev_sorted_filtered_box),
      num_output_box_feature_ * host_filter_count[0] * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_sorted_filtered_label),
                       host_filter_count[0] * sizeof(int)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_sorted_filtered_dir),
                       host_filter_count[0] * sizeof(int)));
  GPU_CHECK(
      cudaMalloc(reinterpret_cast<void**>(&dev_sorted_box_for_nms),
                 num_box_corners_ * host_filter_count[0] * sizeof(float)));

  thrust::device_ptr<float> dev_ptr_filtered_score(dev_filtered_score);
  thrust::host_vector<float> host_filtered_score(host_filter_count[0]);
  thrust::copy(dev_ptr_filtered_score,
               dev_ptr_filtered_score + size_t(host_filter_count[0]),
               host_filtered_score.begin());

  thrust::host_vector<int> host_indexes(host_filter_count[0]);
  thrust::sequence(host_indexes.begin(), host_indexes.end());

  // TODO(chenjiahao): using GPU may cause crash, so use CPU here to sort,
  //  temporarily. Will change to GPU after upgrading CUDA in the future.
  thrust::sort_by_key(host_filtered_score.begin(),
                      host_filtered_score.end(),
                      host_indexes.begin(), thrust::greater<float>());
  GPU_CHECK(cudaMemcpy(dev_indexes,
                       thrust::raw_pointer_cast(host_indexes.data()),
                       host_filter_count[0] * sizeof(int),
                       cudaMemcpyHostToDevice));

  const int num_blocks = DIVUP(host_filter_count[0], num_threads_);
  sort_boxes_by_indexes_kernel<<<num_blocks, num_threads_>>>(
      dev_filtered_box, dev_filtered_label, dev_filtered_dir, dev_box_for_nms,
      dev_indexes, host_filter_count[0], dev_sorted_filtered_box,
      dev_sorted_filtered_label, dev_sorted_filtered_dir,
      dev_sorted_box_for_nms, num_box_corners_, num_output_box_feature_);

  int keep_inds[host_filter_count[0]];
  memset(keep_inds, 0, host_filter_count[0] * sizeof(int));
  int out_num_objects = 0;
  nms_cuda_ptr_->DoNmsCuda(host_filter_count[0], dev_sorted_box_for_nms,
                           keep_inds, &out_num_objects);

  float host_filtered_box[host_filter_count[0] * num_output_box_feature_];
  int host_filtered_label[host_filter_count[0]];
  int host_filtered_dir[host_filter_count[0]];
  GPU_CHECK(
      cudaMemcpy(host_filtered_box, dev_sorted_filtered_box,
                 num_output_box_feature_ * host_filter_count[0] * sizeof(float),
                 cudaMemcpyDeviceToHost));
  GPU_CHECK(cudaMemcpy(host_filtered_label, dev_sorted_filtered_label,
                       host_filter_count[0] * sizeof(int),
                       cudaMemcpyDeviceToHost));
  GPU_CHECK(cudaMemcpy(host_filtered_dir, dev_sorted_filtered_dir,
                       host_filter_count[0] * sizeof(int),
                       cudaMemcpyDeviceToHost));
  for (size_t i = 0; i < out_num_objects; ++i) {
    out_detection->push_back(
        host_filtered_box[keep_inds[i] * num_output_box_feature_ + 0]);
    out_detection->push_back(
        host_filtered_box[keep_inds[i] * num_output_box_feature_ + 1]);
    out_detection->push_back(
        host_filtered_box[keep_inds[i] * num_output_box_feature_ + 2]);
    out_detection->push_back(
        host_filtered_box[keep_inds[i] * num_output_box_feature_ + 3]);
    out_detection->push_back(
        host_filtered_box[keep_inds[i] * num_output_box_feature_ + 4]);
    out_detection->push_back(
        host_filtered_box[keep_inds[i] * num_output_box_feature_ + 5]);

    if (host_filtered_dir[keep_inds[i]] == 0) {
      out_detection->push_back(
          host_filtered_box[keep_inds[i] * num_output_box_feature_ + 6] + M_PI);
    } else {
      out_detection->push_back(
          host_filtered_box[keep_inds[i] * num_output_box_feature_ + 6]);
    }

    out_label->push_back(host_filtered_label[keep_inds[i]]);
  }

  GPU_CHECK(cudaFree(dev_indexes));
  GPU_CHECK(cudaFree(dev_sorted_filtered_box));
  GPU_CHECK(cudaFree(dev_sorted_filtered_label));
  GPU_CHECK(cudaFree(dev_sorted_filtered_dir));
  GPU_CHECK(cudaFree(dev_sorted_box_for_nms));
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
