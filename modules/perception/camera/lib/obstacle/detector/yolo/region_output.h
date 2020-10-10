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
#pragma once

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "modules/perception/base/blob.h"
#include "modules/perception/base/box.h"
#include "modules/perception/base/object.h"
#include "modules/perception/base/object_types.h"
#include "modules/perception/camera/common/math_functions.h"
#include "modules/perception/camera/common/util.h"
#include "modules/perception/camera/lib/obstacle/detector/yolo/proto/yolo.pb.h"

namespace apollo {
namespace perception {
namespace camera {

static const char NormalNMS[] = "NormalNMS";
static const char LinearSoftNMS[] = "LinearSoftNMS";
static const char GuassianSoftNMS[] = "GuassianSoftNMS";
static const char BoxVote[] = "BoxVote";
static const int kBoxBlockSize = 32;
static const int kMaxObjSize = 1000;

struct NormalizedBBox {
  float xmin = -1;
  float ymin = -1;
  float xmax = -1;
  float ymax = -1;
  int label = -1;
  float score = -1;
  float size = -1;
  bool mask = false;

  bool operator()(NormalizedBBox i, NormalizedBBox j) {
    return i.score < j.score;
  }
};

struct BBox3D {
  float h = -1;
  float w = -1;
  float l = -1;
  float alpha = -1;
};

struct AnchorBox {
  float w;
  float h;
};
struct NMSParam {
  float threshold;
  float inter_cls_nms_thresh;
  float inter_cls_conf_thresh;
  float sigma;
  std::string type = BoxVote;
};
struct YoloBlobs {
  std::shared_ptr<base::Blob<float>> det1_loc_blob;
  std::shared_ptr<base::Blob<float>> det1_obj_blob;
  std::shared_ptr<base::Blob<float>> det1_cls_blob;
  std::shared_ptr<base::Blob<float>> det1_ori_conf_blob;
  std::shared_ptr<base::Blob<float>> det1_ori_blob;
  std::shared_ptr<base::Blob<float>> det1_dim_blob;
  std::shared_ptr<base::Blob<float>> det2_loc_blob;
  std::shared_ptr<base::Blob<float>> det2_obj_blob;
  std::shared_ptr<base::Blob<float>> det2_cls_blob;
  std::shared_ptr<base::Blob<float>> det2_ori_conf_blob;
  std::shared_ptr<base::Blob<float>> det2_ori_blob;
  std::shared_ptr<base::Blob<float>> det2_dim_blob;
  std::shared_ptr<base::Blob<float>> det3_loc_blob;
  std::shared_ptr<base::Blob<float>> det3_obj_blob;
  std::shared_ptr<base::Blob<float>> det3_cls_blob;
  std::shared_ptr<base::Blob<float>> det3_ori_conf_blob;
  std::shared_ptr<base::Blob<float>> det3_ori_blob;
  std::shared_ptr<base::Blob<float>> det3_dim_blob;

  std::shared_ptr<base::Blob<float>> lof_blob;
  std::shared_ptr<base::Blob<float>> lor_blob;
  std::shared_ptr<base::Blob<float>> brvis_blob;
  std::shared_ptr<base::Blob<float>> brswt_blob;
  std::shared_ptr<base::Blob<float>> ltvis_blob;
  std::shared_ptr<base::Blob<float>> ltswt_blob;
  std::shared_ptr<base::Blob<float>> rtvis_blob;
  std::shared_ptr<base::Blob<float>> rtswt_blob;
  std::shared_ptr<base::Blob<float>> area_id_blob;
  std::shared_ptr<base::Blob<float>> visible_ratio_blob;
  std::shared_ptr<base::Blob<float>> cut_off_ratio_blob;
  std::shared_ptr<base::Blob<float>> res_box_blob;
  std::shared_ptr<base::Blob<float>> res_cls_blob;
  std::shared_ptr<base::Blob<float>> anchor_blob;
  std::shared_ptr<base::Blob<float>> expand_blob;
};
struct MinDims {
  float min_2d_height = 0.0f;
  float min_3d_height = 0.0f;
  float min_3d_length = 0.0f;
  float min_3d_width = 0.0f;
};

constexpr float minExpPower = -10.0f;
constexpr float maxExpPower = 5.0f;
constexpr int anchorSizeFactor = 2;
constexpr int numScales = 3;

__host__ __device__ float sigmoid_gpu(float x);
__host__ __device__ float bbox_size_gpu(const float *bbox,
                                        const bool normalized);
__host__ __device__ float jaccard_overlap_gpu(const float *bbox1,
                                              const float *bbox2);

template <typename T>
bool sort_score_pair_descend(const std::pair<float, T> &pair1,
                             const std::pair<float, T> &pair2) {
  return pair1.first > pair2.first;
}

void get_max_score_index(const std::vector<float> &scores,
                         const float threshold, const int top_k,
                         std::vector<std::pair<float, int>> *score_index_vec);

float get_bbox_size(const NormalizedBBox &bbox);

void get_intersect_bbox(const NormalizedBBox &bbox1,
                        const NormalizedBBox &bbox2,
                        NormalizedBBox *intersect_bbox);

float get_jaccard_overlap(const NormalizedBBox &bbox1,
                          const NormalizedBBox &bbox2);

void apply_nms(const bool *overlapped, const int num,
               std::vector<int> *indices);

void apply_nms_gpu(const float *bbox_data, const float *conf_data,
                   const std::vector<int> &origin_indices, const int bbox_step,
                   const float confidence_threshold, const int top_k,
                   const float nms_threshold, std::vector<int> *indices,
                   base::Blob<bool> *overlapped, base::Blob<int> *idx_sm,
                   const cudaStream_t &_stream);

void compute_overlapped_by_idx_gpu(const int nthreads, const float *bbox_data,
                                   const float overlap_threshold,
                                   const int *idx, const int num_idx,
                                   bool *overlapped_data,
                                   const cudaStream_t &_stream);

void get_objects_gpu(const YoloBlobs &yolo_blobs, const cudaStream_t &stream,
                     const std::vector<base::ObjectSubType> &types,
                     const NMSParam &nms, const yolo::ModelParam &model_param,
                     float light_vis_conf_threshold,
                     float light_swt_conf_threshold,
                     base::Blob<bool> *overlapped, base::Blob<int> *idx_sm,
                     std::vector<base::ObjectPtr> *objects);

void apply_softnms_fast(const std::vector<NormalizedBBox> &bboxes,
                        std::vector<float> *scores, const float score_threshold,
                        const float nms_threshold, const int top_k,
                        std::vector<int> *indices, bool is_linear,
                        const float sigma);

void apply_boxvoting_fast(std::vector<NormalizedBBox> *bboxes,
                          std::vector<float> *scores,
                          const float conf_threshold, const float nms_threshold,
                          const float sigma, std::vector<int> *indices);

void apply_nms_fast(const std::vector<NormalizedBBox> &bboxes,
                    const std::vector<float> &scores,
                    const float score_threshold, const float nms_threshold,
                    const float eta, const int top_k,
                    std::vector<int> *indices);

void recover_bbox(int roi_w, int roi_h, int offset_y,
                  std::vector<base::ObjectPtr> *objects);

void filter_bbox(const MinDims &min_dims,
                 std::vector<base::ObjectPtr> *objects);

void fill_bbox3d(bool with_bbox3d, base::ObjectPtr obj, const float *bbox);

void fill_frbox(bool with_frbox, base::ObjectPtr obj, const float *bbox);

void fill_lights(bool with_lights, base::ObjectPtr obj, const float *bbox);
void fill_ratios(bool with_ratios, base::ObjectPtr obj, const float *bbox);
void fill_area_id(bool with_flag, base::ObjectPtr obj, const float *data);

void fill_base(base::ObjectPtr obj, const float *bbox);

const float *get_gpu_data(bool flag, const base::Blob<float> &blob);

int get_area_id(float visible_ratios[4]);

}  // namespace camera
}  // namespace perception
}  // namespace apollo
