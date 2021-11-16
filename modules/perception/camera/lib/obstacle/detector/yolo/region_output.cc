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

#include "modules/perception/camera/lib/obstacle/detector/yolo/region_output.h"

#include "cyber/common/log.h"
#include "modules/perception/camera/lib/obstacle/detector/yolo/object_maintainer.h"

namespace apollo {
namespace perception {
namespace camera {

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
                         std::vector<std::pair<float, int>> *score_index_vec) {
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

void apply_softnms_fast(const std::vector<NormalizedBBox> &bboxes,
                        std::vector<float> *scores, const float score_threshold,
                        const float nms_threshold, const int top_k,
                        std::vector<int> *indices, bool is_linear,
                        const float sigma) {
  // Sanity check.
  CHECK_EQ(bboxes.size(), scores->size())
      << "bboxes and scores have different size.";

  // Get top_k scores (with corresponding indices).
  std::vector<std::pair<float, int>> score_index_vec;
  get_max_score_index(*scores, score_threshold, top_k, &score_index_vec);

  // Do nms.
  indices->clear();
  while (!score_index_vec.empty()) {
    auto best_it =
        max_element(std::begin(score_index_vec), std::end(score_index_vec));
    const int best_idx = (*best_it).second;
    score_index_vec.erase(best_it);
    const NormalizedBBox &best_bbox = bboxes[best_idx];
    indices->push_back(best_idx);
    for (std::vector<std::pair<float, int>>::iterator it =
             score_index_vec.begin();
         it != score_index_vec.end();) {
      int cur_idx = it->second;
      const NormalizedBBox &cur_bbox = bboxes[cur_idx];

      float cur_overlap = 0.;
      cur_overlap = get_jaccard_overlap(best_bbox, cur_bbox);
      if (is_linear) {
        (*scores)[cur_idx] *= static_cast<float>((1.0 - cur_overlap));
      } else {
        (*scores)[cur_idx] *=
            static_cast<float>(exp(-1.0 * pow(cur_overlap, 2) / sigma));
      }
      ++it;
    }
  }
}

void apply_boxvoting_fast(std::vector<NormalizedBBox> *bboxes,
                          std::vector<float> *scores,
                          const float conf_threshold, const float nms_threshold,
                          const float sigma, std::vector<int> *indices) {
  if (bboxes->size() == 0) {
    return;
  }
  indices->clear();
  for (int i = 0; i < static_cast<int>(bboxes->size()); i++) {
    (*bboxes)[i].mask = false;
    if ((*scores)[i] > conf_threshold) {
      indices->push_back(i);
    }
  }
  for (int count = 0; count < static_cast<int>(indices->size()); count++) {
    int max_box_idx = 0;

    for (int i = 1; i < static_cast<int>(indices->size()); i++) {
      int idx = indices->at(i);
      if ((*bboxes)[idx].mask) {
        continue;
      }
      if ((*scores)[idx] > (*scores)[max_box_idx]) {
        max_box_idx = idx;
      }
    }

    NormalizedBBox &best_bbox = (*bboxes)[max_box_idx];
    best_bbox.score = (*scores)[max_box_idx];
    best_bbox.mask = true;
    float s_vt = (*scores)[max_box_idx];
    float x1_vt = best_bbox.xmin * s_vt;
    float x2_vt = best_bbox.xmax * s_vt;
    float y1_vt = best_bbox.ymin * s_vt;
    float y2_vt = best_bbox.ymax * s_vt;
    for (int i = 0; i < static_cast<int>(indices->size()); i++) {
      int sub_it = indices->at(i);
      if ((*bboxes)[sub_it].mask) {
        continue;
      }
      float cur_overlap = 0.;
      cur_overlap = get_jaccard_overlap(best_bbox, (*bboxes)[sub_it]);
      if (sigma == 0) {
        (*bboxes)[sub_it].mask = true;
      } else {
        (*scores)[sub_it] *=
            static_cast<float>(exp(-1.0 * pow(cur_overlap, 2) / sigma));
      }
      (*bboxes)[sub_it].score = (*scores)[sub_it];

      // Remove it if necessary
      if (cur_overlap > nms_threshold) {
        float s_vt_cur = (*bboxes)[sub_it].score;
        s_vt += s_vt_cur;
        x1_vt += (*bboxes)[sub_it].xmin * s_vt_cur;
        x2_vt += (*bboxes)[sub_it].xmax * s_vt_cur;
        y1_vt += (*bboxes)[sub_it].ymin * s_vt_cur;
        y2_vt += (*bboxes)[sub_it].ymax * s_vt_cur;
      }
    }
    if (s_vt > 0.0001) {
      (*bboxes)[max_box_idx].xmin = x1_vt / s_vt;
      (*bboxes)[max_box_idx].xmax = x2_vt / s_vt;
      (*bboxes)[max_box_idx].ymin = y1_vt / s_vt;
      (*bboxes)[max_box_idx].ymax = y2_vt / s_vt;
    }
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
  std::vector<std::pair<float, int>> score_index_vec;
  get_max_score_index(scores, score_threshold, top_k, &score_index_vec);

  // Do nms.
  float adaptive_threshold = nms_threshold;
  indices->clear();
  while (!score_index_vec.empty()) {
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

void filter_bbox(const MinDims &min_dims,
                 std::vector<base::ObjectPtr> *objects) {
  int valid_obj_idx = 0;
  int total_obj_idx = 0;
  while (total_obj_idx < static_cast<int>(objects->size())) {
    const auto &obj = (*objects)[total_obj_idx];
    if ((obj->camera_supplement.box.ymax - obj->camera_supplement.box.ymin) >=
            min_dims.min_2d_height &&
        (min_dims.min_3d_height <= 0 ||
         obj->size[2] >= min_dims.min_3d_height) &&
        (min_dims.min_3d_width <= 0 || obj->size[1] >= min_dims.min_3d_width) &&
        (min_dims.min_3d_length <= 0 ||
         obj->size[0] >= min_dims.min_3d_length)) {
      (*objects)[valid_obj_idx] = (*objects)[total_obj_idx];
      ++valid_obj_idx;
    }
    ++total_obj_idx;
  }
  AINFO << valid_obj_idx << " of " << total_obj_idx << " obstacles kept";
  objects->resize(valid_obj_idx);
  AINFO << "Number of detected obstacles: " << objects->size();
}
void recover_bbox(int roi_w, int roi_h, int offset_y,
                  std::vector<base::ObjectPtr> *objects) {
  for (auto &obj : *objects) {
    float xmin = obj->camera_supplement.box.xmin;
    float ymin = obj->camera_supplement.box.ymin;
    float xmax = obj->camera_supplement.box.xmax;
    float ymax = obj->camera_supplement.box.ymax;
    int x = static_cast<int>(xmin * static_cast<float>(roi_w));
    int w = static_cast<int>((xmax - xmin) * static_cast<float>(roi_w));
    int y = static_cast<int>(ymin * static_cast<float>(roi_h)) + offset_y;
    int h = static_cast<int>((ymax - ymin) * static_cast<float>(roi_h));
    base::RectF rect_det(static_cast<float>(x), static_cast<float>(y),
                         static_cast<float>(w), static_cast<float>(h));
    base::RectF rect_img(0, 0, static_cast<float>(roi_w),
                         static_cast<float>(roi_h + offset_y));
    base::RectF rect = rect_det & rect_img;
    obj->camera_supplement.box = rect;

    double eps = 1e-2;

    // Truncation assignment based on bbox positions
    if ((ymin < eps) || (ymax >= 1.0 - eps)) {
      obj->camera_supplement.truncated_vertical = 0.5;
    } else {
      obj->camera_supplement.truncated_vertical = 0.0;
    }
    if ((xmin < eps) || (xmax >= 1.0 - eps)) {
      obj->camera_supplement.truncated_horizontal = 0.5;
    } else {
      obj->camera_supplement.truncated_horizontal = 0.0;
    }

    obj->camera_supplement.front_box.xmin *= static_cast<float>(roi_w);
    obj->camera_supplement.front_box.ymin *= static_cast<float>(roi_h);
    obj->camera_supplement.front_box.xmax *= static_cast<float>(roi_w);
    obj->camera_supplement.front_box.ymax *= static_cast<float>(roi_h);

    obj->camera_supplement.back_box.xmin *= static_cast<float>(roi_w);
    obj->camera_supplement.back_box.ymin *= static_cast<float>(roi_h);
    obj->camera_supplement.back_box.xmax *= static_cast<float>(roi_w);
    obj->camera_supplement.back_box.ymax *= static_cast<float>(roi_h);

    obj->camera_supplement.front_box.ymin += static_cast<float>(offset_y);
    obj->camera_supplement.front_box.ymax += static_cast<float>(offset_y);
    obj->camera_supplement.back_box.ymin += static_cast<float>(offset_y);
    obj->camera_supplement.back_box.ymax += static_cast<float>(offset_y);
  }
}

void fill_base(base::ObjectPtr obj, const float *bbox) {
  obj->camera_supplement.box.xmin = bbox[0];
  obj->camera_supplement.box.ymin = bbox[1];
  obj->camera_supplement.box.xmax = bbox[2];
  obj->camera_supplement.box.ymax = bbox[3];
}

void fill_bbox3d(bool with_box3d, base::ObjectPtr obj, const float *bbox) {
  if (with_box3d) {
    obj->camera_supplement.alpha = bbox[0];
    obj->size[2] = bbox[1];
    obj->size[1] = bbox[2];
    obj->size[0] = bbox[3];
  }
}

void fill_frbox(bool with_frbox, base::ObjectPtr obj, const float *bbox) {
  if (with_frbox) {
    obj->camera_supplement.front_box.xmin = bbox[0];
    obj->camera_supplement.front_box.ymin = bbox[1];
    obj->camera_supplement.front_box.xmax = bbox[2];
    obj->camera_supplement.front_box.ymax = bbox[3];

    obj->camera_supplement.back_box.xmin = bbox[4];
    obj->camera_supplement.back_box.ymin = bbox[5];
    obj->camera_supplement.back_box.xmax = bbox[6];
    obj->camera_supplement.back_box.ymax = bbox[7];
  }
}

void fill_lights(bool with_lights, base::ObjectPtr obj, const float *bbox) {
  if (with_lights) {
    obj->car_light.brake_visible = bbox[0];
    obj->car_light.brake_switch_on = bbox[1];
    obj->car_light.left_turn_visible = bbox[2];
    obj->car_light.left_turn_switch_on = bbox[3];
    obj->car_light.right_turn_visible = bbox[4];
    obj->car_light.right_turn_switch_on = bbox[5];
  }
}

void fill_ratios(bool with_ratios, base::ObjectPtr obj, const float *bbox) {
  if (with_ratios) {
    // visible ratios of face a/b/c/d
    obj->camera_supplement.visible_ratios[0] = bbox[0];
    obj->camera_supplement.visible_ratios[1] = bbox[1];
    obj->camera_supplement.visible_ratios[2] = bbox[2];
    obj->camera_supplement.visible_ratios[3] = bbox[3];

    // cut off on width and length (3D)
    obj->camera_supplement.cut_off_ratios[0] = bbox[4];
    obj->camera_supplement.cut_off_ratios[1] = bbox[5];
    // cut off on left and right side (2D)
    obj->camera_supplement.cut_off_ratios[2] = bbox[6];
    obj->camera_supplement.cut_off_ratios[3] = bbox[7];
  }
}

void fill_area_id(bool with_flag, base::ObjectPtr obj, const float *data) {
  if (with_flag) {
    obj->camera_supplement.area_id = static_cast<int>(data[0]);
    // obj->camera_supplement.area_id_prob = data[1];
  }
}

int get_area_id(float visible_ratios[4]) {
  int area_id = 0;
  int max_face = 0;
  for (int i = 1; i < 4; i++) {
    if (visible_ratios[i] > visible_ratios[max_face]) {
      max_face = i;
    }
  }
  int left_face = (max_face + 1) % 4;
  int right_face = (max_face + 3) % 4;
  const float eps = 1e-3f;
  float max_ratio = visible_ratios[max_face];
  float left_ratio = visible_ratios[left_face];
  float right_ratio = visible_ratios[right_face];
  memset(visible_ratios, 0, 4 * sizeof(visible_ratios[0]));
  if (left_ratio < eps && right_ratio < eps) {
    area_id = (max_face * 2 + 1);
    visible_ratios[max_face] = 1.f;
  } else if (left_ratio > right_ratio) {
    area_id = (max_face * 2 + 2);
    auto &&sum_ratio = left_ratio + max_ratio;
    visible_ratios[max_face] = max_ratio / sum_ratio;
    visible_ratios[left_face] = left_ratio / sum_ratio;
  } else {
    area_id = (max_face * 2);
    if (area_id == 0) {
      area_id = 8;
    }
    auto &&sum_ratio = right_ratio + max_ratio;
    visible_ratios[max_face] = max_ratio / sum_ratio;
    visible_ratios[right_face] = right_ratio / sum_ratio;
  }
  return area_id;
}

const float *get_cpu_data(bool flag, const base::Blob<float> &blob) {
  return flag ? blob.cpu_data() : nullptr;
}

void get_objects_cpu(const YoloBlobs &yolo_blobs, const cudaStream_t &stream,
                     const std::vector<base::ObjectSubType> &types,
                     const NMSParam &nms, const yolo::ModelParam &model_param,
                     float light_vis_conf_threshold,
                     float light_swt_conf_threshold,
                     base::Blob<bool> *overlapped, base::Blob<int> *idx_sm,
                     std::vector<base::ObjectPtr> *objects) {
  std::map<base::ObjectSubType, std::vector<int>> indices;
  std::map<base::ObjectSubType, std::vector<float>> conf_scores;
  int num_kept = 0;
  int num_classes = types.size();
  num_kept = get_objects_gpu(yolo_blobs, stream, types, nms, model_param,
                  light_vis_conf_threshold, light_swt_conf_threshold,
                  overlapped, idx_sm, indices, conf_scores);
  objects->clear();

  if (num_kept == 0) {
    return;
  }

  objects->reserve(num_kept+static_cast<int>(objects->size()));
  const float *cpu_box_data = yolo_blobs.res_box_blob->cpu_data();

  ObjectMaintainer maintainer;
  for (auto it = indices.begin(); it != indices.end(); ++it) {
    base::ObjectSubType label = it->first;
    if (conf_scores.find(label) == conf_scores.end()) {
      // Something bad happened if there are no predictions for current label.
      continue;
    }
    const std::vector<float> &scores = conf_scores.find(label)->second;
    std::vector<int> &indice = it->second;
    for (size_t j = 0; j < indice.size(); ++j) {
      int idx = indice[j];
      const float *bbox = cpu_box_data + idx * kBoxBlockSize;
      if (scores[idx] < model_param.confidence_threshold()) {
        continue;
      }

      base::ObjectPtr obj = nullptr;
      obj.reset(new base::Object);
      obj->type = base::kSubType2TypeMap.at(label);
      obj->sub_type = label;
      obj->type_probs.assign(
          static_cast<int>(base::ObjectType::MAX_OBJECT_TYPE), 0);
      obj->sub_type_probs.assign(
          static_cast<int>(base::ObjectSubType::MAX_OBJECT_TYPE), 0);
      float total = 1e-5;
      for (int k = 0; k < num_classes; ++k) {
        auto &vis_type_k = types[k];
        auto &obj_type_k = base::kSubType2TypeMap.at(vis_type_k);
        auto &conf_score = conf_scores[vis_type_k][idx];
        obj->type_probs[static_cast<int>(obj_type_k)] += conf_score;
        obj->sub_type_probs[static_cast<int>(vis_type_k)] = conf_score;
        total += conf_score;
      }
      obj->confidence = obj->type_probs[static_cast<int>(obj->type)];
      for (size_t k = 0; k < obj->type_probs.size(); ++k) {
        obj->type_probs[k] /= total;
      }
      fill_base(obj, bbox);
      fill_bbox3d(model_param.with_box3d(), obj, bbox + 4);
      fill_frbox(model_param.with_frbox(), obj, bbox + 8);
      fill_lights(model_param.with_lights(), obj, bbox + 16);
      fill_ratios(model_param.with_ratios(), obj, bbox + 22);
      fill_area_id(model_param.num_areas() > 0, obj, bbox + 30);

      if (maintainer.Add(idx, obj)) {
        objects->push_back(obj);
      }
    }
  }
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
