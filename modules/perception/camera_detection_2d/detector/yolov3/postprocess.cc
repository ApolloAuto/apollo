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

#include "modules/perception/camera_detection_2d/detector/yolov3/postprocess.h"

#include <unordered_map>

#include "cyber/common/log.h"
#include "modules/perception/common/base/nms.h"

namespace apollo {
namespace perception {
namespace camera {

// Car,Van,Truck,Pedestrian,Person_sitting,Cyclist,Tram,Misc
constexpr int kClassOfObstacles = 8;
// 2dcenter_x、2dcenter_y、2d_box_w、2d_box_h、confidence/score、
// 3dbox-h、3dbox-w、3dbox-l、aplha、8 classes
constexpr int kScoreIndex = 4;
// 2dcenter_x、2dcenter_y、2d_box_w、2d_box_h、confidence/score、
// 3dbox-h、3dbox-w、3dbox-l、aplha、conf_3d、object_score、object_label
constexpr int kTypeIndex = 10;
constexpr int kLabelIndex = 11;

const base::ObjectSubType kYoloSubType[] = {
    base::ObjectSubType::CAR,         base::ObjectSubType::VAN,
    base::ObjectSubType::TRUCK,       base::ObjectSubType::BUS,
    base::ObjectSubType::PEDESTRIAN,  base::ObjectSubType::CYCLIST,
    base::ObjectSubType::TRAFFICCONE, base::ObjectSubType::UNKNOWN,
};

void GetAllObjects(const float *data, const yolov3::ModelParam &model_param,
                   std::vector<std::vector<float>> *objects) {
  ACHECK(objects != nullptr);

  objects->clear();
  int object_num = model_param.output_object_num();
  int object_size = model_param.output_object_size();

  for (int num = 0; num < object_num; ++num) {
    float score = data[kScoreIndex + object_size * num];
    if (score < model_param.confidence_threshold()) {
      continue;
    }

    std::vector<float> object;
    std::vector<float> scores;
    for (int j = 0; j < object_size; ++j) {
      // index 5 ~ 12
      if (j > kScoreIndex && j <= (kScoreIndex + kClassOfObstacles)) {
        scores.push_back(data[j + object_size * num]);
      } else {  // index 0 ~ 4 + 13 ~ 17
        // 2dcenter_x、2dcenter_y、2d_box_w、2d_box_h、confidence/score、
        // 3dbox-h、3dbox-w、3dbox-l、aplha
        object.push_back(data[j + object_size * num]);
      }
    }

    auto max_iter = std::max_element(scores.begin(), scores.end());
    int max_idx = std::distance(scores.begin(), max_iter);
    object.push_back(*max_iter);
    object.push_back(max_idx);
    objects->push_back(object);
  }
}

float ComputeOverlap(const base::Rect<float> &a, const base::Rect<float> &b) {
  base::Rect<float> intersection = a & b;
  base::Rect<float> unionsection = a | b;
  if (unionsection.Area() == 0) return 0.0f;
  return intersection.Area() / unionsection.Area();
}

void ApplyNms(const std::vector<std::vector<float>> &detect_objects,
              const float nms_threshold,
              std::vector<std::vector<float>> *detect_nms) {
  ACHECK(detect_nms != nullptr);

  std::unordered_map<int, std::vector<std::vector<float>>> objects_map;
  for (const auto &detection : detect_objects) {
    int label = static_cast<int>(detection[kLabelIndex]);
    objects_map[label].push_back(detection);
  }

  for (const auto &iter : objects_map) {
    std::vector<float> scores;
    std::vector<base::Rect<float>> bboxes;
    std::vector<std::vector<float>> objects = iter.second;
    for (const std::vector<float> &data : objects) {
      scores.push_back(data[kScoreIndex]);
      float width = data[2];
      float height = data[3];
      float xmin = data[0] - width / 2;
      float ymin = data[1] - height / 2;
      bboxes.emplace_back(xmin, ymin, width, height);
    }

    std::vector<int> indices;
    // TODO(lordon): check last one is 0
    Nms(bboxes, scores, 0, nms_threshold, 0, 0, &indices, ComputeOverlap, 0);

    for (const int &idx : indices) {
      detect_nms->push_back(objects[idx]);
    }
  }
}

void FillBase(const std::vector<float> &detect, const int width,
              const int height, const int image_width, const int image_height,
              base::ObjectPtr obj) {
  // yolo image size is (416, 416), raw image is (1080, 1920)
  int x1 = static_cast<int>(detect[0]) - std::floor(detect[2] / 2.0);
  int y1 = static_cast<int>(detect[1]) - std::floor(detect[3] / 2.0);
  int x2 = static_cast<int>(detect[0]) + std::floor(detect[2] / 2.0);
  int y2 = static_cast<int>(detect[1]) + std::floor(detect[3] / 2.0);

  int pad_x =
      std::floor(std::max(image_height - image_width, 0) *
                 (static_cast<float>(width) /
                  static_cast<float>(std::max(image_width, image_height))));
  int pad_y =
      std::floor(std::max(image_width - image_height, 0) *
                 (static_cast<float>(height) /
                  static_cast<float>(std::max(image_width, image_height))));

  float unpad_h = static_cast<float>(height - pad_y);
  float unpad_w = static_cast<float>(width - pad_x);

  int box_h = static_cast<int>((static_cast<float>((y2 - y1)) / unpad_h) *
                               image_height);
  int box_w =
      static_cast<int>((static_cast<float>((x2 - x1)) / unpad_w) * image_width);
  // xmin and ymin in raw image
  x1 = static_cast<int>(
      ((static_cast<float>(x1) - static_cast<float>(pad_x) / 2.0) / unpad_w) *
      image_width);
  y1 = static_cast<int>(
      ((static_cast<float>(y1) - static_cast<float>(pad_y) / 2.0) / unpad_h) *
      image_height);

  obj->camera_supplement.box.xmin = x1;
  obj->camera_supplement.box.ymin = y1;
  obj->camera_supplement.box.xmax = x1 + box_w;
  obj->camera_supplement.box.ymax = y1 + box_h;
}

void FillBbox3d(bool with_box3d, const std::vector<float> &detect,
                base::ObjectPtr obj) {
  if (with_box3d) {
    // length, width, height
    obj->size[0] = detect[7];
    obj->size[1] = detect[6];
    obj->size[2] = detect[5];
    obj->camera_supplement.alpha = detect[8];
  }
}

float BboxIOU(const std::vector<float> &box1, const std::vector<float> &box2) {
  const int center_x_index = 0;
  const int center_y_index = 1;
  const int width_index = 2;
  const int height_index = 3;

  int b1_x1 = static_cast<int>(box1[center_x_index]) -
              std::floor(box1[width_index] / 2.0);
  int b1_y1 = static_cast<int>(box1[center_y_index]) -
              std::floor(box1[height_index] / 2.0);
  int b1_x2 = static_cast<int>(box1[center_x_index]) +
              std::floor(box1[width_index] / 2.0);
  int b1_y2 = static_cast<int>(box1[center_y_index]) +
              std::floor(box1[height_index] / 2.0);

  int b2_x1 = static_cast<int>(box2[center_x_index]) -
              std::floor(box2[width_index] / 2.0);
  int b2_y1 = static_cast<int>(box2[center_y_index]) -
              std::floor(box2[height_index] / 2.0);
  int b2_x2 = static_cast<int>(box2[center_x_index]) +
              std::floor(box2[width_index] / 2.0);
  int b2_y2 = static_cast<int>(box2[center_y_index]) +
              std::floor(box2[height_index] / 2.0);

  // get the corrdinates of the intersection rectangle
  int inter_rect_x1 = std::max(b1_x1, b2_x1);
  int inter_rect_y1 = std::max(b1_y1, b2_y1);
  int inter_rect_x2 = std::min(b1_x2, b2_x2);
  int inter_rect_y2 = std::min(b1_y2, b2_y2);

  // Intersection area
  float inter_area =
      static_cast<float>(clamp(inter_rect_x2 - inter_rect_x1 + 1, 0, INT_MAX) *
                         clamp(inter_rect_y2 - inter_rect_y1 + 1, 0, INT_MAX));
  // Union Area
  float b1_area = (b1_x2 - b1_x1 + 1) * (b1_y2 - b1_y1 + 1);
  float b2_area = (b2_x2 - b2_x1 + 1) * (b2_y2 - b2_y1 + 1);

  float iou = inter_area / (b1_area + b2_area - inter_area + 1e-3);
  return iou;
}

void NmsForObjects(const std::vector<std::vector<float>> &detect_objects,
                   const float nms_threshold,
                   std::vector<std::vector<float>> *detect_nms) {
  // get all unique labels
  std::set<int> object_class_set;
  for (size_t i = 0; i < detect_objects.size(); i++) {
    object_class_set.insert(static_cast<int>(detect_objects[i][kLabelIndex]));
  }
  // for each label, iterate all objects
  for (auto ite = object_class_set.begin(); ite != object_class_set.end();
       ite++) {
    std::vector<std::vector<float>> obj;
    for (size_t i = 0; i < detect_objects.size(); i++) {
      if (*ite == detect_objects[i][kLabelIndex]) {
        obj.push_back(detect_objects[i]);
      }
    }
    // according to confidence
    std::sort(obj.begin(), obj.end(),
              [](std::vector<float> obj_1, std::vector<float> obj_2) {
                return obj_1[kScoreIndex] > obj_2[kScoreIndex];
              });
    // store nms result
    std::vector<std::vector<float>> max_detections;
    // Perform non-maximum suppression
    while (obj.size() != 0) {
      max_detections.push_back(obj[0]);
      // if (obj.size() == 1) break;
      size_t index = 1;
      // Get the IOUs for all boxes with lower confidence
      while (index < obj.size()) {
        float iou =
            BboxIOU(max_detections[max_detections.size() - 1], obj[index]);
        if (iou >= nms_threshold) {
          obj.erase(obj.begin() + index);
        } else {
          index++;
        }
      }
      obj.erase(obj.begin());
    }
    // save all max_detections to objects_nms
    for (auto i : max_detections) {
      detect_nms->push_back(i);
    }
  }
}

void GetYolov3ObjectsCpu(const std::shared_ptr<base::Blob<float>> &objects_blob,
                         const yolov3::ModelParam &model_param,
                         const yolov3::NMSParam &nms_param, const int width,
                         const int height, const int image_width,
                         const int image_height,
                         std::vector<base::ObjectPtr> *objects) {
  ACHECK(objects_blob != nullptr);
  ACHECK(objects != nullptr);

  const float *detect_data = objects_blob->cpu_data();
  ACHECK(detect_data != nullptr);

  std::vector<std::vector<float>> detect_objects;
  GetAllObjects(detect_data, model_param, &detect_objects);

  std::vector<std::vector<float>> detect_nms;
  // ApplyNms(detect_objects, nms_param.threshold(), &detect_nms);
  NmsForObjects(detect_objects, nms_param.threshold(), &detect_nms);

  AINFO << "Yolo detect before nms: " << detect_objects.size();
  AINFO << "After nms number of objects is: " << detect_nms.size();

  // center_x, center_y, width, height, object_confidence, h, w, l, alpha,
  // class_score, class_label
  objects->clear();
  for (const std::vector<float> &detect : detect_nms) {
    // Todo(zero):
    const int label = detect[kLabelIndex];

    if (7 <= label) {
      continue;
    }

    base::ObjectPtr obj = nullptr;
    obj.reset(new base::Object);

    obj->sub_type = kYoloSubType[label];
    obj->type = base::kSubType2TypeMap.at(obj->sub_type);

    obj->type_probs.assign(static_cast<int>(base::ObjectType::MAX_OBJECT_TYPE),
                           0);
    obj->sub_type_probs.assign(
        static_cast<int>(base::ObjectSubType::MAX_OBJECT_TYPE), 0);
    obj->type_probs[static_cast<int>(obj->type)] = detect[kTypeIndex];
    obj->sub_type_probs[static_cast<int>(obj->sub_type)] = detect[kTypeIndex];
    obj->confidence = detect[kScoreIndex];

    FillBase(detect, width, height, image_width, image_height, obj);
    // use 3d regress result to get 3d bbox
    FillBbox3d(model_param.with_box3d(), detect, obj);

    objects->push_back(obj);
  }
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
