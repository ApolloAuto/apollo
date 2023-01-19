/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include <algorithm>
#include <functional>
#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "boost/iterator/counting_iterator.hpp"
#include "thrust/functional.h"
#include "thrust/sort.h"

#include "modules/perception/base/object_types.h"
#include "modules/perception/camera/lib/obstacle/detector/yolo/object_maintainer.h"
#include "modules/perception/camera/lib/obstacle/detector/yolo/region_output.h"

namespace apollo {
namespace perception {
namespace camera {

__host__ __device__ float sigmoid_gpu(float x) { return 1.0 / (1.0 + exp(-x)); }

__host__ __device__ float bbox_size_gpu(const float *bbox,
                                        const bool normalized) {
  if (bbox[2] <= bbox[0] || bbox[3] <= bbox[1]) {
    // If bbox is invalid (e.g. xmax < xmin or ymax < ymin), return 0.
    return 0.f;  // NOLINT
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
  if (bbox2[0] > bbox1[2] || bbox2[2] < bbox1[0] || bbox2[1] > bbox1[3] ||
      bbox2[3] < bbox1[1]) {
    return float(0.);  // NOLINT
  } else {
    const float inter_xmin = fmaxf(bbox1[0], bbox2[0]);
    const float inter_ymin = fmaxf(bbox1[1], bbox2[1]);
    const float inter_xmax = fminf(bbox1[2], bbox2[2]);
    const float inter_ymax = fminf(bbox1[3], bbox2[3]);

    const float inter_width = inter_xmax - inter_xmin;
    const float inter_height = inter_ymax - inter_ymin;
    const float inter_size = inter_width * inter_height;

    const float bbox1_size = bbox_size_gpu(bbox1, true);
    const float bbox2_size = bbox_size_gpu(bbox2, true);

    return inter_size / (bbox1_size + bbox2_size - inter_size);
  }
}

__global__ void get_object_kernel(
    int n, const float *loc_data, const float *obj_data, const float *cls_data,
    const float *ori_data, const float *dim_data, const float *lof_data,
    const float *lor_data, const float *area_id_data,
    const float *visible_ratio_data, const float *cut_off_ratio_data,
    const float *brvis_data, const float *brswt_data, const float *ltvis_data,
    const float *ltswt_data, const float *rtvis_data, const float *rtswt_data,
    const float *anchor_data, const float *expand_data, int width, int height,
    int num_anchors, int num_classes, float confidence_threshold,
    float light_vis_conf_threshold, float light_swt_conf_threshold,
    bool with_box3d, bool with_frbox, bool with_lights, bool with_ratios,
    bool multi_scale, int num_areas, float *res_box_data, float *res_cls_data,
    int res_cls_offset, int all_scales_num_candidates) {
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < (n);
       i += blockDim.x * gridDim.x) {
    int box_block = kBoxBlockSize;

    int idx = i;
    int c = idx % num_anchors;
    idx = idx / num_anchors;
    int w = idx % width;
    idx = idx / width;
    int h = idx;
    int loc_index = (h * width + w) * num_anchors + c;
    int offset_loc = loc_index * 4;
    int offset_cls = loc_index * num_classes;
    float scale = obj_data[loc_index];
    float cx = (w + sigmoid_gpu(loc_data[offset_loc + 0])) / width;
    float cy = (h + sigmoid_gpu(loc_data[offset_loc + 1])) / height;
    float hw =
        exp(fmaxf(minExpPower, fminf(loc_data[offset_loc + 2], maxExpPower))) *
        anchor_data[2 * c] / width * 0.5;
    float hh =
        exp(fmaxf(minExpPower, fminf(loc_data[offset_loc + 3], maxExpPower))) *
        anchor_data[2 * c + 1] / height * 0.5;

    float max_prob = 0.f;
    int max_index = 0;
    for (int k = 0; k < num_classes; ++k) {
      float prob = cls_data[offset_cls + k] * scale;
      res_cls_data[k * all_scales_num_candidates + res_cls_offset + i] = prob;
      if (prob > max_prob) {
        max_prob = prob;
        max_index = k;
      }
    }
    res_cls_data[num_classes * all_scales_num_candidates + res_cls_offset + i] =
        max_prob;

    auto &&dst_ptr = res_box_data + i * box_block;
    hw += expand_data[max_index];
    dst_ptr[0] = cx - hw;
    dst_ptr[1] = cy - hh;
    dst_ptr[2] = cx + hw;
    dst_ptr[3] = cy + hh;

    if (with_box3d) {
      int offset_ori = loc_index * 2;
      dst_ptr[4] = atan2(ori_data[offset_ori + 1], ori_data[offset_ori]);

      int offset_dim = loc_index * 3;
      if (multi_scale) {
        offset_dim = loc_index * num_classes * 3 + max_index * 3;
      }
      dst_ptr[5] = dim_data[offset_dim + 0];
      dst_ptr[6] = dim_data[offset_dim + 1];
      dst_ptr[7] = dim_data[offset_dim + 2];
    }

    if (with_frbox) {
      {
        int offset_lof = loc_index * 4;
        auto &&src_ptr = lof_data + offset_lof;
        auto sb_x = src_ptr[0] * hw * 2 + cx;
        auto sb_y = src_ptr[1] * hh * 2 + cy;
        auto sb_hw = exp(src_ptr[2]) * hw;
        auto sb_hh = exp(src_ptr[3]) * hh;
        dst_ptr[8] = sb_x - sb_hw;
        dst_ptr[9] = sb_y - sb_hh;
        dst_ptr[10] = sb_x + sb_hw;
        dst_ptr[11] = sb_y + sb_hh;
      }

      {
        int offset_lor = loc_index * 4;
        auto &&src_ptr = lor_data + offset_lor;
        auto sb_x = src_ptr[0] * hw * 2 + cx;
        auto sb_y = src_ptr[1] * hh * 2 + cy;
        auto sb_hw = exp(src_ptr[2]) * hw;
        auto sb_hh = exp(src_ptr[3]) * hh;
        dst_ptr[12] = sb_x - sb_hw;
        dst_ptr[13] = sb_y - sb_hh;
        dst_ptr[14] = sb_x + sb_hw;
        dst_ptr[15] = sb_y + sb_hh;
      }
    }

    if (with_lights) {
      dst_ptr[16] = sigmoid_gpu(brvis_data[loc_index]);
      dst_ptr[17] = sigmoid_gpu(brswt_data[loc_index]);
      dst_ptr[18] = sigmoid_gpu(ltvis_data[loc_index]);
      dst_ptr[19] = sigmoid_gpu(ltswt_data[loc_index]);
      dst_ptr[20] = sigmoid_gpu(rtvis_data[loc_index]);
      dst_ptr[21] = sigmoid_gpu(rtswt_data[loc_index]);

      dst_ptr[16] = dst_ptr[16] > light_vis_conf_threshold ? dst_ptr[16] : 0;
      dst_ptr[18] = dst_ptr[18] > light_vis_conf_threshold ? dst_ptr[18] : 0;
      dst_ptr[20] = dst_ptr[20] > light_vis_conf_threshold ? dst_ptr[20] : 0;

      float swt_score = 0;
      swt_score = dst_ptr[16] * dst_ptr[17];
      dst_ptr[17] = swt_score > light_swt_conf_threshold ? swt_score : 0;

      swt_score = dst_ptr[18] * dst_ptr[19];
      dst_ptr[19] = swt_score > light_swt_conf_threshold ? swt_score : 0;

      swt_score = dst_ptr[20] * dst_ptr[21];
      dst_ptr[21] = swt_score > light_swt_conf_threshold ? swt_score : 0;
    }

    if (with_ratios) {
      // 0~3: cos2, left, visa, visb
      auto vis_pred = visible_ratio_data + loc_index * 4;
      auto vis_ptr = dst_ptr + 22;
      vis_ptr[0] = vis_ptr[1] = vis_ptr[2] = vis_ptr[3] = 0;
      const float hi_th = 0.75;
      const float lo_th = 1.f - hi_th;
      if (vis_pred[2] >= hi_th && vis_pred[3] >= hi_th) {  // 2 (1, 3)
        vis_ptr[0] = vis_pred[0];
        vis_ptr[1] = 1 - vis_pred[0];
      } else if (vis_pred[2] <= lo_th && vis_pred[3] >= hi_th) {  // 4 (3, 5)
        vis_ptr[2] = vis_pred[0];
        vis_ptr[1] = 1 - vis_pred[0];
      } else if (vis_pred[2] <= lo_th && vis_pred[3] <= lo_th) {  // 6 (5, 7)
        vis_ptr[2] = vis_pred[0];
        vis_ptr[3] = 1 - vis_pred[0];
      } else if (vis_pred[2] >= hi_th && vis_pred[3] <= lo_th) {  // 8 (7, 1)
        vis_ptr[0] = vis_pred[0];
        vis_ptr[3] = 1 - vis_pred[0];
      } else {
        vis_ptr[2] = vis_pred[0];
        if (vis_pred[1] > 0.5) {
          vis_ptr[1] = 1 - vis_pred[0];
        } else {
          vis_ptr[3] = 1 - vis_pred[0];
        }
      }

      int offset_cut = loc_index * 4;
      dst_ptr[26] = cut_off_ratio_data[offset_cut + 0];
      dst_ptr[27] = cut_off_ratio_data[offset_cut + 1];
      dst_ptr[28] = cut_off_ratio_data[offset_cut + 2];
      dst_ptr[29] = cut_off_ratio_data[offset_cut + 3];
    }

    if (num_areas > 0) {
      int offset_area_id = loc_index * num_areas;
      int max_area_id = 0;
      for (int area_id = 1; area_id < num_areas; ++area_id) {
        if (area_id_data[offset_area_id + area_id] >
            area_id_data[offset_area_id + max_area_id]) {
          max_area_id = area_id;
        }
      }
      dst_ptr[30] = max_area_id + 1;
      dst_ptr[31] = area_id_data[offset_area_id + max_area_id];
    }
  }
}

__global__ void get_rois_kernel(int num_bboxes, const float *loc_data,
                                const float *obj_data, const float *anchor_data,
                                int width, int height, int num_anchors,
                                float confidence_threshold, float *conf_data,
                                float *bbox_data) {
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < num_bboxes) {
    int offset_obj = idx;
    int offset_loc = idx * 4;

    int c = idx % num_anchors;
    idx /= num_anchors;
    int w = idx % width;
    idx /= width;
    int h = idx;

    float cx = (w + sigmoid_gpu(loc_data[offset_loc + 0])) / width;
    float cy = (h + sigmoid_gpu(loc_data[offset_loc + 1])) / height;
    float hw =
        exp(loc_data[offset_loc + 2]) * anchor_data[2 * c + 0] / width * 0.5;
    float hh =
        exp(loc_data[offset_loc + 3]) * anchor_data[2 * c + 1] / height * 0.5;

    const float &conf = obj_data[offset_obj];
    conf_data[offset_obj] = conf > confidence_threshold ? conf : 0;

    auto &&curr_bbox = bbox_data + offset_loc;
    curr_bbox[0] = cx - hw;
    curr_bbox[1] = cy - hh;
    curr_bbox[2] = cx + hw;
    curr_bbox[3] = cy + hh;
  }
}

__global__ void compute_overlapped_by_idx_kernel(
    const int nthreads, const float *bbox_data, const int bbox_step,
    const float overlap_threshold, const int *idx, const int num_idx,
    bool *overlapped_data) {
  for (int index = blockIdx.x * blockDim.x + threadIdx.x; index < (nthreads);
       index += blockDim.x * gridDim.x) {
    const int j = index % num_idx;
    const int i = index / num_idx;
    if (i == j) {
      // Ignore same bbox.
      return;
    }
    // Compute overlap between i-th bbox and j-th bbox.
    const int start_loc_i = idx[i] * bbox_step;
    const int start_loc_j = idx[j] * bbox_step;
    const float overlap =
        jaccard_overlap_gpu(bbox_data + start_loc_i, bbox_data + start_loc_j);
    overlapped_data[index] = overlap > overlap_threshold;
  }
}

void compute_overlapped_by_idx_gpu(const int nthreads, const float *bbox_data,
                                   const int bbox_step,
                                   const float overlap_threshold,
                                   const int *idx, const int num_idx,
                                   bool *overlapped_data,
                                   const cudaStream_t &stream) {
  // NOLINT_NEXT_LINE(whitespace/operators)
  const int thread_size = 512;
  int block_size = (nthreads + thread_size - 1) / thread_size;
  compute_overlapped_by_idx_kernel<<<block_size, thread_size, 0, stream>>>(
      nthreads, bbox_data, bbox_step, overlap_threshold, idx, num_idx,
      overlapped_data);
}

void apply_nms_gpu(const float *bbox_data, const float *conf_data,
                   const std::vector<int> &origin_indices, const int bbox_step,
                   const float confidence_threshold, const int top_k,
                   const float nms_threshold, std::vector<int> *indices,
                   base::Blob<bool> *overlapped, base::Blob<int> *idx_sm,
                   const cudaStream_t &stream) {
  // Keep part of detections whose scores are higher than confidence threshold.
  std::vector<int> idx;
  std::vector<float> confidences;
  for (auto i : origin_indices) {
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
  int *idx_data = (idx_sm->mutable_cpu_data());
  std::copy(idx.begin(), idx.begin() + num_remain, idx_data);

  overlapped->Reshape(std::vector<int>{num_remain, num_remain});
  bool *overlapped_data = (overlapped->mutable_gpu_data());

  compute_overlapped_by_idx_gpu(overlapped->count(), bbox_data, bbox_step,
                                nms_threshold, idx_sm->gpu_data(), num_remain,
                                overlapped_data, stream);

  // Do non-maximum suppression based on overlapped results.
  const bool *overlapped_results = (const bool *)overlapped->cpu_data();
  std::vector<int> selected_indices;

  apply_nms(overlapped_results, num_remain, &selected_indices);
  // Put back the selected information.
  for (size_t i = 0; i < selected_indices.size(); ++i) {
    indices->push_back(idx[selected_indices[i]]);
  }
}

void apply_nms(const bool *overlapped, const int num,
               std::vector<int> *indices) {
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

    for (std::vector<int>::iterator it = index_vec.begin();
         it != index_vec.end();) {
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

const float *get_gpu_data(bool flag, const base::Blob<float> &blob) {
  return flag ? blob.gpu_data() : nullptr;
}

int get_objects_gpu(const YoloBlobs &yolo_blobs, const cudaStream_t &stream,
    const std::vector<base::ObjectSubType> &types,
    const NMSParam &nms, const yolo::ModelParam &model_param,
    float light_vis_conf_threshold,
    float light_swt_conf_threshold,
    base::Blob<bool> *overlapped, base::Blob<int> *idx_sm,
    const std::map<base::ObjectSubType, std::vector<int>> &indices_cns,
    const std::map<base::ObjectSubType, std::vector<float>> &conf_scores_cns) {
  auto& indices = const_cast<std::map<base::ObjectSubType,
                      std::vector<int>>& >(indices_cns);
  auto& conf_scores = const_cast<std::map<base::ObjectSubType,
                      std::vector<float>>& >(conf_scores_cns);

  bool multi_scale = false;
  if (yolo_blobs.det2_obj_blob) {
    multi_scale = true;
  }
  int num_classes = types.size();
  int batch = yolo_blobs.det1_obj_blob->shape(0);
  int num_anchor = yolo_blobs.anchor_blob->shape(2);
  int num_anchor_per_scale = num_anchor;
  if (multi_scale) {
    num_anchor_per_scale /= numScales;
  }
  CHECK_EQ(batch, 1) << "batch size should be 1!";

  std::vector<int> height_vec, width_vec, num_candidates_vec;
  height_vec.push_back(yolo_blobs.det1_obj_blob->shape(1));
  width_vec.push_back(yolo_blobs.det1_obj_blob->shape(2));
  if (multi_scale) {
    height_vec.push_back(yolo_blobs.det2_obj_blob->shape(1));
    height_vec.push_back(yolo_blobs.det3_obj_blob->shape(1));
    width_vec.push_back(yolo_blobs.det2_obj_blob->shape(2));
    width_vec.push_back(yolo_blobs.det3_obj_blob->shape(2));
  }
  for (size_t i = 0; i < height_vec.size(); i++) {
    num_candidates_vec.push_back(height_vec[i] * width_vec[i] *
                                 num_anchor_per_scale);
  }

  const float *loc_data_vec[3] = {
      yolo_blobs.det1_loc_blob->gpu_data(),
      yolo_blobs.det2_loc_blob ? yolo_blobs.det2_loc_blob->gpu_data() : nullptr,
      yolo_blobs.det3_loc_blob ? yolo_blobs.det3_loc_blob->gpu_data()
                               : nullptr};
  const float *obj_data_vec[3] = {
      yolo_blobs.det1_obj_blob->gpu_data(),
      yolo_blobs.det2_obj_blob ? yolo_blobs.det2_obj_blob->gpu_data() : nullptr,
      yolo_blobs.det3_obj_blob ? yolo_blobs.det3_obj_blob->gpu_data()
                               : nullptr};
  const float *cls_data_vec[3] = {
      yolo_blobs.det1_cls_blob->gpu_data(),
      yolo_blobs.det2_cls_blob ? yolo_blobs.det2_cls_blob->gpu_data() : nullptr,
      yolo_blobs.det3_cls_blob ? yolo_blobs.det3_cls_blob->gpu_data()
                               : nullptr};
  const float *ori_data_vec[3] = {
      get_gpu_data(model_param.with_box3d(), *yolo_blobs.det1_ori_blob),
      multi_scale
          ? get_gpu_data(model_param.with_box3d(), *yolo_blobs.det2_ori_blob)
          : nullptr,
      multi_scale
          ? get_gpu_data(model_param.with_box3d(), *yolo_blobs.det3_ori_blob)
          : nullptr};
  const float *dim_data_vec[3] = {
      get_gpu_data(model_param.with_box3d(), *yolo_blobs.det1_dim_blob),
      multi_scale
          ? get_gpu_data(model_param.with_box3d(), *yolo_blobs.det2_dim_blob)
          : nullptr,
      multi_scale
          ? get_gpu_data(model_param.with_box3d(), *yolo_blobs.det3_dim_blob)
          : nullptr};

  // TODO[KaWai]: add 3 scale frbox data and light data.
  const float *lof_data =
      get_gpu_data(model_param.with_frbox(), *yolo_blobs.lof_blob);
  const float *lor_data =
      get_gpu_data(model_param.with_frbox(), *yolo_blobs.lor_blob);

  const float *area_id_data =
      get_gpu_data(model_param.num_areas() > 0, *yolo_blobs.area_id_blob);
  const float *visible_ratio_data =
      get_gpu_data(model_param.with_ratios(), *yolo_blobs.visible_ratio_blob);
  const float *cut_off_ratio_data =
      get_gpu_data(model_param.with_ratios(), *yolo_blobs.cut_off_ratio_blob);

  const auto &with_lights = model_param.with_lights();
  const float *brvis_data = get_gpu_data(with_lights, *yolo_blobs.brvis_blob);
  const float *brswt_data = get_gpu_data(with_lights, *yolo_blobs.brswt_blob);
  const float *ltvis_data = get_gpu_data(with_lights, *yolo_blobs.ltvis_blob);
  const float *ltswt_data = get_gpu_data(with_lights, *yolo_blobs.ltswt_blob);
  const float *rtvis_data = get_gpu_data(with_lights, *yolo_blobs.rtvis_blob);
  const float *rtswt_data = get_gpu_data(with_lights, *yolo_blobs.rtswt_blob);

  int all_scales_num_candidates = 0;
  for (size_t i = 0; i < num_candidates_vec.size(); i++) {
    all_scales_num_candidates += num_candidates_vec[i];
  }
  yolo_blobs.res_box_blob->Reshape(
      std::vector<int>{1, 1, all_scales_num_candidates, kBoxBlockSize});
  yolo_blobs.res_cls_blob->Reshape(
      std::vector<int>{1, 1, num_classes + 1, all_scales_num_candidates});

  float *res_box_data = yolo_blobs.res_box_blob->mutable_gpu_data();
  float *res_cls_data = yolo_blobs.res_cls_blob->mutable_gpu_data();
  const int thread_size = 512;
  // TODO[KaWai]: use different stream to process scales in parallel.
  int num_candidates_offset = 0;
  for (int i = 0; i < num_candidates_vec.size(); i++) {
    int block_size = (num_candidates_vec[i] + thread_size - 1) / thread_size;
    const float *loc_data = loc_data_vec[i];
    const float *obj_data = obj_data_vec[i];
    const float *cls_data = cls_data_vec[i];
    const float *ori_data = ori_data_vec[i];
    const float *dim_data = dim_data_vec[i];
    const float *anchor_data =
        yolo_blobs.anchor_blob->gpu_data() + num_anchor_per_scale * 2 * i;
    const float *expand_data = yolo_blobs.expand_blob->gpu_data();
    const int width = width_vec[i];
    const int height = height_vec[i];
    get_object_kernel<<<block_size, thread_size, 0, stream>>>(
        num_candidates_vec[i], loc_data, obj_data, cls_data, ori_data, dim_data,
        lof_data, lor_data, area_id_data, visible_ratio_data,
        cut_off_ratio_data, brvis_data, brswt_data, ltvis_data, ltswt_data,
        rtvis_data, rtswt_data, anchor_data, yolo_blobs.expand_blob->gpu_data(),
        width, height, num_anchor_per_scale, num_classes,
        model_param.confidence_threshold(), light_vis_conf_threshold,
        light_swt_conf_threshold, model_param.with_box3d(),
        model_param.with_frbox(), model_param.with_lights(),
        model_param.with_ratios(), multi_scale, model_param.num_areas(),
        res_box_data + num_candidates_offset * kBoxBlockSize, res_cls_data,
        num_candidates_offset, all_scales_num_candidates);
    cudaStreamSynchronize(stream);
    num_candidates_offset += num_candidates_vec[i];
  }
  const float *cpu_cls_data = yolo_blobs.res_cls_blob->cpu_data();

  std::vector<int> all_indices(all_scales_num_candidates);
  std::iota(all_indices.begin(), all_indices.end(), 0);
  std::vector<int> rest_indices;

  int top_k = idx_sm->count();
  int num_kept = 0;
  // inter-cls NMS
  apply_nms_gpu(
      res_box_data, cpu_cls_data + num_classes * all_scales_num_candidates,
      all_indices, kBoxBlockSize, nms.inter_cls_conf_thresh, top_k,
      nms.inter_cls_nms_thresh, &rest_indices, overlapped, idx_sm, stream);
  for (int k = 0; k < num_classes; ++k) {
    apply_nms_gpu(res_box_data, cpu_cls_data + k * all_scales_num_candidates,
                  rest_indices, kBoxBlockSize,
                  model_param.confidence_threshold(), top_k, nms.threshold,
                  &(indices[types[k]]), overlapped, idx_sm, stream);
    num_kept += indices[types[k]].size();
    std::vector<float> conf_score(
        cpu_cls_data + k * all_scales_num_candidates,
        cpu_cls_data + (k + 1) * all_scales_num_candidates);
    conf_scores.insert(std::make_pair(types[k], conf_score));
    cudaStreamSynchronize(stream);
  }

  return num_kept;
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
