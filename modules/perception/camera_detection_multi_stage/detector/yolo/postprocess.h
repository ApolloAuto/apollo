/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "modules/perception/camera_detection_multi_stage/detector/yolo/proto/model_param.pb.h"

#include "modules/perception/common/base/blob.h"
#include "modules/perception/common/base/object.h"
#include "modules/perception/common/base/object_types.h"

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

  std::shared_ptr<base::Blob<float>> feat_blob;
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
/**
 * @brief Get the max score index object
 *
 * @param scores  the scores array
 * @param threshold threshold value
 * @param top_k top k
 * @param score_index_vec output index array
 */
void get_max_score_index(const std::vector<float> &scores,
                         const float threshold, const int top_k,
                         std::vector<std::pair<float, int>> *score_index_vec);

float get_bbox_size(const NormalizedBBox &bbox);
/**
 * @brief Get the intersect bbox object
 *
 * @param bbox1 [in] first bbox
 * @param bbox2 [in] second bbox
 * @param intersect_bbox [out] intersect bbox
 */
void get_intersect_bbox(const NormalizedBBox &bbox1,
                        const NormalizedBBox &bbox2,
                        NormalizedBBox *intersect_bbox);
/**
 * @brief Get the jaccard overlap object
 *
 * @param bbox1 [in] first bbox
 * @param bbox2 [in] second bbox
 * @return float jaccard overlap
 */
float get_jaccard_overlap(const NormalizedBBox &bbox1,
                          const NormalizedBBox &bbox2);
/**
 * @brief Get all boxes with score larger than threshold
 *
 * @param overlapped  overlap result
 * @param num  number of boxes with score larger than threshold
 * @param indices indices of boxes with score larger than threshold
 */
void apply_nms(const bool *overlapped, const int num,
               std::vector<int> *indices);
/**
 * @brief Get all boxes with score larger than threshold
 *
 * @param bbox_data bbox data
 * @param conf_data conf data
 * @param origin_indices origin indices
 * @param bbox_step bbox step
 * @param confidence_threshold onfidence threshold
 * @param top_k top k
 * @param nms_threshold nms threshold
 * @param indices indices of boxes with score larger than threshold
 * @param overlapped  overlap result
 * @param idx_sm indices of boxes with score larger than threshold
 * @param _stream  stream
 */
void apply_nms_gpu(const float *bbox_data, const float *conf_data,
                   const std::vector<int> &origin_indices, const int bbox_step,
                   const float confidence_threshold, const int top_k,
                   const float nms_threshold, std::vector<int> *indices,
                   base::Blob<bool> *overlapped, base::Blob<int> *idx_sm,
                   const cudaStream_t &_stream);
/**
 * @brief Get all boxes with score larger than threshold
 *
 * @param nthreads number of threads
 * @param bbox_data  bbox data
 * @param overlap_threshold   overlap threshold
 * @param idx  bbox index
 * @param num_idx  number of bbox index
 * @param overlapped_data overlap result
 * @param _stream  stream
 */
void compute_overlapped_by_idx_gpu(const int nthreads, const float *bbox_data,
                                   const float overlap_threshold,
                                   const int *idx, const int num_idx,
                                   bool *overlapped_data,
                                   const cudaStream_t &_stream);
/**
 * @brief Get the objects gpu object
 *
 * @param yolo_blobs  YOLO blobs
 * @param stream  Stream to use
 * @param types   Type of object
 * @param nms   NMS type
 * @param model_param   Model's parameter
 * @param light_vis_conf_threshold  Light vis confidence threshold
 * @param light_swt_conf_threshold  Light swt confidence threshold
 * @param overlapped  Overlapped objects
 * @param idx_sm Indices of objects in blob
 * @param indices  Indices of objects in blob
 * @param conf_scores  Confidence scores of objects
 * @return int
 */
int get_objects_gpu(
    const YoloBlobs &yolo_blobs, const cudaStream_t &stream,
    const std::vector<base::ObjectSubType> &types, const NMSParam &nms,
    const yolo::ModelParam &model_param, float light_vis_conf_threshold,
    float light_swt_conf_threshold, base::Blob<bool> *overlapped,
    base::Blob<int> *idx_sm,
    const std::map<base::ObjectSubType, std::vector<int>> &indices,
    const std::map<base::ObjectSubType, std::vector<float>> &conf_scores);
/**
 * @brief Get the objects cpu object
 *
 * @param yolo_blobs  YOLO blobs
 * @param stream  CUDA stream
 * @param types  object types
 * @param nms  nms params
 * @param model_param   model params
 * @param light_vis_conf_threshold  light vis conf threshold
 * @param light_swt_conf_threshold  light swt conf threshold
 * @param overlapped  overlapped blob
 * @param idx_sm idx sm blob
 * @param objects  object blob
 */
void get_objects_cpu(const YoloBlobs &yolo_blobs, const cudaStream_t &stream,
                     const std::vector<base::ObjectSubType> &types,
                     const NMSParam &nms, const yolo::ModelParam &model_param,
                     float light_vis_conf_threshold,
                     float light_swt_conf_threshold,
                     base::Blob<bool> *overlapped, base::Blob<int> *idx_sm,
                     std::vector<base::ObjectPtr> *objects);
/**
 * @brief softnms for objects
 *
 * @param bboxes  vector of bboxes
 * @param scores vector of scores
 * @param score_threshold threshold for scores
 * @param nms_threshold threshold for nms
 * @param top_k top k scores
 * @param indices indices of object
 * @param is_linear true if is linear
 * @param sigma sigma for gaussian kernel
 */
void apply_softnms_fast(const std::vector<NormalizedBBox> &bboxes,
                        std::vector<float> *scores, const float score_threshold,
                        const float nms_threshold, const int top_k,
                        std::vector<int> *indices, bool is_linear,
                        const float sigma);
/**
 * @brief filter target detection results based on given thresholds and
 * parameters.
 *
 * @param bboxes  2d bbox results
 * @param scores  scores results of bbox
 * @param conf_threshold confidence threshold
 * @param nms_threshold nms threshold
 * @param sigma sigma parameter
 * @param indices indices of matched objects
 */
void apply_boxvoting_fast(std::vector<NormalizedBBox> *bboxes,
                          std::vector<float> *scores,
                          const float conf_threshold, const float nms_threshold,
                          const float sigma, std::vector<int> *indices);
/**
 * @brief nms for objects
 *
 * @param bboxes input bounding boxes
 * @param scores input scores, size sames to boxes
 * @param score_threshold score threshold
 * @param nms_threshold NMS threshold
 * @param eta eta parameter
 * @param top_k top_k parameter for nms
 * @param indices  indices of objects to keep
 */
void apply_nms_fast(const std::vector<NormalizedBBox> &bboxes,
                    const std::vector<float> &scores,
                    const float score_threshold, const float nms_threshold,
                    const float eta, const int top_k,
                    std::vector<int> *indices);
/**
 * @brief recover detect bbox result to raw image
 *
 * @param roi_w roi width
 * @param roi_h roi height
 * @param offset_y offset y
 * @param objects detection result
 */
void recover_bbox(int roi_w, int roi_h, int offset_y,
                  std::vector<base::ObjectPtr> *objects);
/**
 * @brief filter out the objects in the object array based on the given minimum
 * dimension conditions and retain the objects that meet the conditions
 *
 * @param min_dims minimum dimensions of the bounding box
 * @param objects list of objects
 */
void filter_bbox(const MinDims &min_dims,
                 std::vector<base::ObjectPtr> *objects);
/**
 * @brief add alpha h w l to object
 *
 * @param with_bbox3d whether to add alpha h w l to object
 * @param obj object
 * @param bbox  the bbox of object
 */
void fill_bbox3d(bool with_bbox3d, base::ObjectPtr obj, const float *bbox);
/**
 * @brief add car front and backward bbox value to object
 *
 * @param with_frbox whether to add car front and backward bbox value to object
 * @param obj object
 * @param bbox  the bbox of object
 */
void fill_frbox(bool with_frbox, base::ObjectPtr obj, const float *bbox);
/**
 * @brief add car lights value to object
 *
 * @param with_lights lights info
 * @param obj object to be filled
 * @param bbox  the bbox of object
 */
void fill_lights(bool with_lights, base::ObjectPtr obj, const float *bbox);
/**
 * @brief add ratio value to object
 *
 * @param with_ratios ratio value will be added to object
 * @param obj object to be filled
 * @param bbox the bbox of object
 */
void fill_ratios(bool with_ratios, base::ObjectPtr obj, const float *bbox);
/**
 * @brief add area id value to object
 *
 * @param with_flag true: area id is exist
 * @param obj  object result
 * @param data area id
 */
void fill_area_id(bool with_flag, base::ObjectPtr obj, const float *data);
/**
 * @brief add bbox value to object
 *
 * @param obj object result
 * @param bbox [N, 4]
 */
void fill_base(base::ObjectPtr obj, const float *bbox);
/**
 * @brief Get the gpu data object
 *
 * @param flag flag to get the gpu data object
 * @param blob blob to get the gpu data object
 * @return const float*  the gpu data object
 */
const float *get_gpu_data(bool flag, const base::Blob<float> &blob);
/**
 * @brief calculate the area id based on the visible ratios
 *
 * @param visible_ratios visible ratios
 * @return int area_id
 */
int get_area_id(float visible_ratios[4]);

}  // namespace camera
}  // namespace perception
}  // namespace apollo
