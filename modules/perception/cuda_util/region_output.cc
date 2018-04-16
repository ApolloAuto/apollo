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

#include "modules/perception/cuda_util/region_output.h"

#include <algorithm>
#include <map>
#include <vector>

#include "boost/iterator/counting_iterator.hpp"
#include "opencv2/opencv.hpp"

namespace apollo {
namespace perception {

void get_intersect_bbox(const NormalizedBBox &bbox1,
                        const NormalizedBBox &bbox2,
                        NormalizedBBox *intersect_bbox) {
  if (bbox2.xmin > bbox1.xmax || bbox2.xmax < bbox1.xmin ||
      bbox2.ymin > bbox1.ymax || bbox2.ymax < bbox1.ymin) {
    // Return [0, 0, 0, 0] if there is no intersection.
    intersect_bbox->xmin = 0;
    intersect_bbox->ymin = 0;
    intersect_bbox->xmax = 0;
    intersect_bbox->ymax = 0;
  } else {
    intersect_bbox->xmin = std::max(bbox1.xmin, bbox2.xmin);
    intersect_bbox->ymin = std::max(bbox1.ymin, bbox2.ymin);
    intersect_bbox->xmax = std::min(bbox1.xmax, bbox2.xmax);
    intersect_bbox->ymax = std::min(bbox1.ymax, bbox2.ymax);
  }
}

float get_bbox_size(const NormalizedBBox &bbox) {
  if (bbox.xmax < bbox.xmin || bbox.ymax < bbox.ymin) {
    // If bbox is invalid (e.g. xmax < xmin or ymax < ymin), return 0.
    return 0;
  } else {
    if (bbox.size >= 0) {
      return bbox.size;
    } else {
      float width = bbox.xmax - bbox.xmin;
      float height = bbox.ymax - bbox.ymin;
      return width * height;
    }
  }
}

float get_jaccard_overlap(const NormalizedBBox &bbox1,
                          const NormalizedBBox &bbox2) {
  NormalizedBBox intersect_bbox;
  get_intersect_bbox(bbox1, bbox2, &intersect_bbox);
  float intersect_width = 0.f;
  float intersect_height = 0.f;
  intersect_width = intersect_bbox.xmax - intersect_bbox.xmin;
  intersect_height = intersect_bbox.ymax - intersect_bbox.ymin;

  if (intersect_width > 0 && intersect_height > 0) {
    float intersect_size = intersect_width * intersect_height;
    float bbox1_size = get_bbox_size(bbox1);
    float bbox2_size = get_bbox_size(bbox2);
    return intersect_size / (bbox1_size + bbox2_size - intersect_size);
  } else {
    return 0.;
  }
}

void get_max_score_index(const std::vector<float> &scores,
                         const float threshold, const int top_k,
                         std::vector<std::pair<float, int> > *score_index_vec) {
  // Generate index score pairs.
  for (int i = 0; i < static_cast<int>(scores.size()); ++i) {
    if (scores[i] > threshold) {
      score_index_vec->push_back(std::make_pair(scores[i], i));
    }
  }

  // Sort the score pair according to the scores in descending order
  std::stable_sort(score_index_vec->begin(), score_index_vec->end(),
                   sort_score_pair_descend<int>);

  // Keep top_k scores if needed.
  if (top_k > -1 && top_k < static_cast<int>(score_index_vec->size())) {
    score_index_vec->resize(top_k);
  }
}
void apply_nms_fast(const std::vector<NormalizedBBox> &bboxes,
                    const std::vector<float> &scores,
                    const float score_threshold, const float nms_threshold,
                    const float eta, const int top_k,
                    std::vector<int> *indices) {
  // Sanity check.
  CHECK_EQ(bboxes.size(), scores.size())
      << "bboxes and scores have different size.";

  // Get top_k scores (with corresponding indices).
  std::vector<std::pair<float, int> > score_index_vec;
  get_max_score_index(scores, score_threshold, top_k, &score_index_vec);

  // Do nms.
  float adaptive_threshold = nms_threshold;
  indices->clear();
  while (score_index_vec.size() != 0) {
    const int idx = score_index_vec.front().second;
    bool keep = true;
    for (int k = 0; k < static_cast<int>(indices->size()); ++k) {
      if (keep) {
        const int kept_idx = (*indices)[k];
        float overlap = get_jaccard_overlap(bboxes[idx], bboxes[kept_idx]);
        keep = overlap <= adaptive_threshold;
      } else {
        break;
      }
    }
    if (keep) {
      indices->push_back(idx);
    }
    score_index_vec.erase(score_index_vec.begin());
    if (keep && eta < 1 && adaptive_threshold > 0.5) {
      adaptive_threshold *= eta;
    }
  }
}

void cross_class_merge(std::vector<int> *indices_ref,
                       std::vector<int> *indices_target,
                       std::vector<NormalizedBBox> bboxes, float scale) {
  for (int i = 0; i < static_cast<int>(indices_ref->size()); i++) {
    int ref_idx = indices_ref->at(i);
    NormalizedBBox &bbox_ref = bboxes[ref_idx];
    for (std::vector<int>::iterator it = indices_target->begin();
         it != indices_target->end();) {
      int target_idx = *it;
      NormalizedBBox &bbox_target = bboxes[target_idx];
      NormalizedBBox intersection;
      get_intersect_bbox(bbox_ref, bbox_target, &intersection);
      intersection.size = get_bbox_size(intersection);
      bbox_target.size = get_bbox_size(bbox_target);
      bbox_ref.size = get_bbox_size(bbox_ref);
      if (intersection.size > bbox_target.size * scale &&
          bbox_target.ymax <= bbox_ref.ymax) {
        it = indices_target->erase(it);
      } else {
        it++;
      }
    }
  }
}

}  // namespace perception
}  // namespace apollo
