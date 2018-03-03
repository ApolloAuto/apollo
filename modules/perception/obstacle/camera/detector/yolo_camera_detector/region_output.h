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

#define MODULES_PERCEPTION_OBSTACLE_CAMERA_DETECTOR_REGION_OUTPUT_H_

//#include <boost/property_tree/json_parser.hpp>
//#include <boost/property_tree/ptree.hpp>
//#include <boost/regex.hpp>

#include <map>
#include <string>
#include <utility>
#include <vector>
#include "modules/common/log.h
#include <obstacle/camera/detector/common/util.h>

#include <obstacle/camera/common/util.h>
#include "modules/caffe/blob.hpp"
#include "modules/caffe/layer.hpp"
#include "modules/caffe/proto/caffe.pb.h"

#include "modules/infer.h"
#include "modules/layer.h"
#include "modules/net.h"

#include "modules/obstacle/camera/common/visual_object.h"

namespace apollo {
namespace perception {
namespace obstacle {
static const char NormalNMS[] = "NormalNMS";
static const char LinearSoftNMS[] = "LinearSoftNMS";
static const char GuassianSoftNMS[] = "GuassianSoftNMS";
static const char BoxVote[] = "BoxVote";
static const int s_box_block_size = 16;

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
  std::string type = BoxVote;
  float sigma;
};
template <typename Dtype>
inline Dtype sigmoid(Dtype x) {
  return 1.0 / (1.0 + exp(-x));
}

template <typename Dtype>
NormalizedBBox get_region_box(const std::vector<AnchorBox> anchor_boxes,
                              const Dtype *loc_data, int i, int j, int w, int h,
                              int n, int index) {
  NormalizedBBox bbox;
  Dtype cx = (i + sigmoid(loc_data[index + 0])) / w;
  Dtype cy = (j + sigmoid(loc_data[index + 1])) / h;
  Dtype hw = exp(loc_data[index + 2]) * anchor_boxes[n].w / w * 0.5;
  Dtype hh = exp(loc_data[index + 3]) * anchor_boxes[n].h / h * 0.5;
  bbox.xmin = cx - hw;
  bbox.ymin = cy - hh;
  bbox.xmax = cx + hw;
  bbox.ymax = cy + hh;
  return bbox;
}

template <typename T>
bool sort_score_pair_descend(const std::pair<float, T> &pair1,
                             const std::pair<float, T> &pair2) {
  return pair1.first > pair2.first;
}

void get_max_score_index(const std::vector<float> &scores,
                         const float threshold, const int top_k,
                         std::vector<std::pair<float, int> > *score_index_vec);

float get_bbox_size(const NormalizedBBox &bbox);

void get_intersect_bbox(const NormalizedBBox &bbox1,
                        const NormalizedBBox &bbox2,
                        NormalizedBBox *intersect_bbox);

float get_jaccard_overlap(const NormalizedBBox &bbox1,
                          const NormalizedBBox &bbox2);

void apply_nms(const bool *overlapped, const int num,
               std::vector<int> *indices);
void apply_nms_gpu(const float *bbox_data, const float *conf_data,
                   const int num_bboxes, const float confidence_threshold,
                   const int top_k, const float nms_threshold,
                   std::vector<int> *indices,
                   std::shared_ptr<SyncedMemory> overlappe,
                   std::shared_ptr<SyncedMemory> idx_sm);
void compute_overlapped_by_idx_gpu(const int nthreads, const float *bbox_data,
                                   const float overlap_threshold,
                                   const int *idx, const int num_idx,
                                   bool *overlapped_data);

void apply_nms_fast(const std::vector<NormalizedBBox> &bboxes,
                    const std::vector<float> &scores,
                    const float score_threshold, const float nms_threshold,
                    const float eta, const int top_k,
                    std::vector<int> *indices);

void cross_class_merge(std::vector<int> *indices_ref,
                       std::vector<int> *indices_target,
                       std::vector<NormalizedBBox> bboxes, float scale);

void recover_bbox(int roi_w, int roi_h, int offset_y,
                  std::vector<VisualObjectPtr> *objects);

}  // namespace obstacle
}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_DETECTOR_REGION_OUTPUT_H_
