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

#include "modules/perception/camera_detection_multi_stage/detector/yolox3d/postprocess.h"

#include <map>
#include <unordered_map>
#include <utility>

#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace camera {

// Car,Truck,Van,Bus,Pedestrian,Cyclist,TrafficCone,Uknown
constexpr int kClassOfObstacles = 8;
// 2dcenter_x、2dcenter_y、2d_box_w、2d_box_h、score、object_label
constexpr int kScoreIndex = 4;
constexpr int kLabelIndex = 5;

const base::ObjectSubType kYoloSubTypeYolox[] = {
        base::ObjectSubType::CAR,
        base::ObjectSubType::TRUCK,
        base::ObjectSubType::VAN,
        base::ObjectSubType::BUS,
        base::ObjectSubType::PEDESTRIAN,
        base::ObjectSubType::CYCLIST,
        base::ObjectSubType::TRAFFICCONE,
        base::ObjectSubType::UNKNOWN,
};

struct Object {
    cv::Rect_<float> rect;
    int label;
    float prob;
};

struct GridAndStride {
    int grid0;
    int grid1;
    int stride;
};

static void generate_grids_and_stride(std::vector<int> &strides, std::vector<GridAndStride> &grid_strides) {
    for (auto stride : strides) {
        int num_grid_y = 640 / stride;
        int num_grid_x = 640 / stride;
        for (int g1 = 0; g1 < num_grid_y; g1++) {
            for (int g0 = 0; g0 < num_grid_x; g0++) {
                grid_strides.push_back((GridAndStride){g0, g1, stride});
            }
        }
    }
}

static void generate_yolox3d_proposals(
        std::vector<GridAndStride> grid_strides,
        const float *feat_blob,
        float prob_threshold,
        std::vector<Object> &objects) {
    const int num_anchors = grid_strides.size();  // 8400

    for (int anchor_idx = 0; anchor_idx < num_anchors; anchor_idx++) {
        const int grid0 = grid_strides[anchor_idx].grid0;
        const int grid1 = grid_strides[anchor_idx].grid1;
        const int stride = grid_strides[anchor_idx].stride;

        const int basic_pos = anchor_idx * (kClassOfObstacles + 5);

        // yolox3d/models/yolo_head.py decode logic
        float x_center = (feat_blob[basic_pos + 0] + grid0) * stride;
        float y_center = (feat_blob[basic_pos + 1] + grid1) * stride;
        float w = exp(feat_blob[basic_pos + 2]) * stride;
        float h = exp(feat_blob[basic_pos + 3]) * stride;
        float x0 = x_center - w * 0.5f;
        float y0 = y_center - h * 0.5f;

        if (w - 10 < 0 || h - 10 < 0) {
            continue;
        }

        std::vector<float> scores;
        scores.clear();
        float box_objectness = feat_blob[basic_pos + kScoreIndex];
        for (int class_idx = 0; class_idx < kClassOfObstacles; class_idx++) {
            float box_cls_score = feat_blob[basic_pos + kLabelIndex + class_idx];
            float box_prob = box_objectness * box_cls_score;
            scores.push_back(box_prob);
        }
        auto max_iter = std::max_element(scores.begin(), scores.end());
        int max_idx = std::distance(scores.begin(), max_iter);

        if (*max_iter > prob_threshold) {
            Object obj;
            obj.rect.x = x0;
            obj.rect.y = y0;
            obj.rect.width = w;
            obj.rect.height = h;
            obj.label = max_idx;
            obj.prob = *max_iter;
            objects.push_back(obj);
        }
    }  // point anchor loop
}

static inline float intersection_area(const Object &a, const Object &b) {
    cv::Rect_<float> inter = a.rect & b.rect;
    return inter.area();
}

static void nms_sorted_bboxes(const std::vector<Object> &faceobjects, std::vector<int> &picked, float nms_threshold) {
    picked.clear();

    const int n = faceobjects.size();

    std::vector<float> areas(n);
    for (int i = 0; i < n; i++) {
        areas[i] = faceobjects[i].rect.area();
    }

    for (int i = 0; i < n; i++) {
        const Object &a = faceobjects[i];

        int keep = 1;
        for (int j = 0; j < static_cast<int>(picked.size()); j++) {
            const Object &b = faceobjects[picked[j]];

            // intersection over union
            float inter_area = intersection_area(a, b);
            float union_area = areas[i] + areas[picked[j]] - inter_area;
            // float IoU = inter_area / union_area
            if (inter_area / union_area > nms_threshold)
                keep = 0;
        }

        if (keep)
            picked.push_back(i);
    }
}

static void qsort_descent_inplace(std::vector<Object> &faceobjects, int left, int right) {
    int i = left;
    int j = right;
    float p = faceobjects[(left + right) / 2].prob;

    while (i <= j) {
        while (faceobjects[i].prob > p) {
            i++;
        }

        while (faceobjects[j].prob < p) {
            j--;
        }

        if (i <= j) {
            // swap
            std::swap(faceobjects[i], faceobjects[j]);

            i++;
            j--;
        }
    }

#pragma omp parallel sections
    {
#pragma omp section
        {
            if (left < j)
                qsort_descent_inplace(faceobjects, left, j);
        }
#pragma omp section
        {
            if (i < right)
                qsort_descent_inplace(faceobjects, i, right);
        }
    }
}

static void qsort_descent_inplace(std::vector<Object> &objects) {
    if (objects.empty())
        return;

    qsort_descent_inplace(objects, 0, objects.size() - 1);
}

void YoloxGetAllObjects(
        const float *data,
        const yolox3d::ModelParam &model_param,
        const float scale,
        std::vector<std::vector<float>> *objects_out) {
    objects_out->clear();
    std::vector<Object> objects;

    std::vector<Object> proposals;
    std::vector<int> strides = {8, 16, 32};
    std::vector<GridAndStride> grid_strides;
    generate_grids_and_stride(strides, grid_strides);
    generate_yolox3d_proposals(grid_strides, data, model_param.confidence_threshold(), proposals);
    qsort_descent_inplace(proposals);

    std::vector<int> picked;
    nms_sorted_bboxes(proposals, picked, model_param.nms_param().threshold());
    int count = picked.size();

    objects.resize(count);
    for (int i = 0; i < count; i++) {
        objects[i] = proposals[picked[i]];

        // adjust offset to original unpadded
        float x0 = (objects[i].rect.x) / scale;
        float y0 = (objects[i].rect.y) / scale;
        float x1 = (objects[i].rect.x + objects[i].rect.width) / scale;
        float y1 = (objects[i].rect.y + objects[i].rect.height) / scale;

        // clip
        x0 = std::max(std::min(x0, static_cast<float>(1920 - 1)), 0.f);
        y0 = std::max(std::min(y0, static_cast<float>(1920 - 1)), 0.f);
        x1 = std::max(std::min(x1, static_cast<float>(1920 - 1)), 0.f);
        y1 = std::max(std::min(y1, static_cast<float>(1920 - 1)), 0.f);

        objects[i].rect.x = x0;
        objects[i].rect.y = y0;
        objects[i].rect.width = x1 - x0;
        objects[i].rect.height = y1 - y0;

        std::vector<float> object_temp;
        object_temp.push_back(x0);       // left x
        object_temp.push_back(y0);       // left y
        object_temp.push_back(x1 - x0);  // w
        object_temp.push_back(y1 - y0);  // h

        object_temp.push_back(objects[i].prob);   // score
        object_temp.push_back(objects[i].label);  // class label

        objects_out->push_back(object_temp);
    }
}

float ComputeYoloxOverlap(const base::Rect<float> &a, const base::Rect<float> &b) {
    base::Rect<float> intersection = a & b;
    base::Rect<float> unionsection = a | b;
    if (unionsection.Area() == 0)
        return 0.0f;
    return intersection.Area() / unionsection.Area();
}

void YoloxFillBase(
        const std::vector<float> &detect,
        const int width,
        const int height,
        const int image_width,
        const int image_height,
        base::ObjectPtr obj) {
    // yolo image size is (416, 416), raw image is (1080, 1920)
    obj->camera_supplement.box.xmin = detect[0];
    obj->camera_supplement.box.ymin = detect[1];
    obj->camera_supplement.box.xmax = detect[0] + detect[2];
    obj->camera_supplement.box.ymax = detect[1] + detect[3];
}

void YoloxFillBbox3d(const yolox3d::ModelParam &model_param, const std::vector<float> &detect, base::ObjectPtr obj) {
    auto obj_l = 0.0;
    auto obj_w = 0.0;
    auto obj_h = 0.0;

    if (model_param.with_box3d()) {
        if (base ::ObjectType ::VEHICLE == base::kSubType2TypeMap.at(obj->sub_type)) {
            obj_l = model_param.car_template().l();
            obj_w = model_param.car_template().w();
            obj_h = model_param.car_template().h();
            if ("TRUCK" == base::kSubType2NameMap.at(obj->sub_type)) {
                obj_l = model_param.car_template().l() * 1.5;
                obj_w = model_param.car_template().w() * 1.5;
                obj_h = model_param.car_template().h() * 1.5;
            } else if ("BUS" == base::kSubType2NameMap.at(obj->sub_type)) {
                obj_l = model_param.car_template().l() * 3;
                obj_w = model_param.car_template().w() * 1.5;
                obj_h = model_param.car_template().h() * 2.0;
            }
        } else if (base ::ObjectType ::PEDESTRIAN == base::kSubType2TypeMap.at(obj->sub_type)) {
            obj_l = model_param.ped_template().l();
            obj_w = model_param.ped_template().w();
            obj_h = model_param.ped_template().h();
        } else if (base ::ObjectType ::BICYCLE == base::kSubType2TypeMap.at(obj->sub_type)) {
            obj_l = model_param.cyclist_template().l();
            obj_w = model_param.cyclist_template().w();
            obj_h = model_param.cyclist_template().h();
        } else if (base ::ObjectType ::UNKNOWN_UNMOVABLE == base::kSubType2TypeMap.at(obj->sub_type)) {
            obj_l = model_param.trafficcone_template().l();
            obj_w = model_param.trafficcone_template().w();
            obj_h = model_param.trafficcone_template().h();
        }

        // length, width, height
        obj->size[0] = obj_l;
        obj->size[1] = obj_w;
        obj->size[2] = obj_h;
        obj->camera_supplement.alpha = detect[8];
    }
}

float YoloxBboxIOU(const std::vector<float> &box1, const std::vector<float> &box2) {
    const int center_x_index = 0;
    const int center_y_index = 1;
    const int width_index = 2;
    const int height_index = 3;

    int b1_x1 = static_cast<int>(box1[center_x_index]) - std::floor(box1[width_index] / 2.0);
    int b1_y1 = static_cast<int>(box1[center_y_index]) - std::floor(box1[height_index] / 2.0);
    int b1_x2 = static_cast<int>(box1[center_x_index]) + std::floor(box1[width_index] / 2.0);
    int b1_y2 = static_cast<int>(box1[center_y_index]) + std::floor(box1[height_index] / 2.0);

    int b2_x1 = static_cast<int>(box2[center_x_index]) - std::floor(box2[width_index] / 2.0);
    int b2_y1 = static_cast<int>(box2[center_y_index]) - std::floor(box2[height_index] / 2.0);
    int b2_x2 = static_cast<int>(box2[center_x_index]) + std::floor(box2[width_index] / 2.0);
    int b2_y2 = static_cast<int>(box2[center_y_index]) + std::floor(box2[height_index] / 2.0);

    // get the corrdinates of the intersection rectangle
    int inter_rect_x1 = std::max(b1_x1, b2_x1);
    int inter_rect_y1 = std::max(b1_y1, b2_y1);
    int inter_rect_x2 = std::min(b1_x2, b2_x2);
    int inter_rect_y2 = std::min(b1_y2, b2_y2);

    // Intersection area
    float inter_area = static_cast<float>(
            Yoloxclamp(inter_rect_x2 - inter_rect_x1 + 1, 0, INT_MAX)
            * Yoloxclamp(inter_rect_y2 - inter_rect_y1 + 1, 0, INT_MAX));
    // Union Area
    float b1_area = (b1_x2 - b1_x1 + 1) * (b1_y2 - b1_y1 + 1);
    float b2_area = (b2_x2 - b2_x1 + 1) * (b2_y2 - b2_y1 + 1);

    float iou = inter_area / (b1_area + b2_area - inter_area + 1e-3);
    return iou;
}

void YoloxTruncated(base::ObjectPtr obj, const int image_width, const int image_height) {
    const float boundary_len = 20.0;  // TODO(lordon): 20 piexl move it to conf
    float min_x = static_cast<float>(boundary_len);
    float min_y = static_cast<float>(boundary_len);
    float max_x = static_cast<float>(image_width - boundary_len);
    float max_y = static_cast<float>(image_height - boundary_len);
    double eps = 1e-2;

    if (obj->camera_supplement.box.xmin <= min_x || obj->camera_supplement.box.xmax >= max_x
        || obj->camera_supplement.box.ymin <= min_y || obj->camera_supplement.box.ymax >= max_y) {
        // Truncation assignment based on bbox positions
        if ((obj->camera_supplement.box.ymin < eps) || (obj->camera_supplement.box.ymax >= 1.0 - eps)) {
            obj->camera_supplement.truncated_vertical = 0.5;
        } else {
            obj->camera_supplement.truncated_vertical = 0.0;
        }
        if ((obj->camera_supplement.box.xmin < eps) || (obj->camera_supplement.box.xmax >= 1.0 - eps)) {
            obj->camera_supplement.truncated_horizontal = 0.5;
        } else {
            obj->camera_supplement.truncated_horizontal = 0.0;
        }
    }
}

void YoloxGetObjectsCpu(
        const std::shared_ptr<base::Blob<float>> &objects_blob,
        const yolox3d::ModelParam &model_param,
        const yolox3d::NMSParam &nms_param,
        const int width,
        const int height,
        const int image_width,
        const int image_height,
        std::vector<base::ObjectPtr> *objects) {
    ACHECK(objects_blob != nullptr);
    ACHECK(objects != nullptr);

    const float *detect_data = objects_blob->cpu_data();
    ACHECK(detect_data != nullptr);

    float scale = std::min(
            model_param.resize().width() / (image_width * 1.0), model_param.resize().height() / (image_height * 1.0));

    std::vector<std::vector<float>> detect_objects;
    YoloxGetAllObjects(detect_data, model_param, scale, &detect_objects);
    AINFO << "Yolox objects after nms is " << detect_objects.size();

    // center_x, center_y, width, height, class_score, class_label
    objects->clear();
    for (const std::vector<float> &detect : detect_objects) {
        const int label = detect[kLabelIndex];
        // ignore unknown
        if (7 <= label) {
            continue;
        }

        base::ObjectPtr obj = nullptr;
        obj.reset(new base::Object);

        obj->sub_type = kYoloSubTypeYolox[label];
        obj->type = base::kSubType2TypeMap.at(obj->sub_type);
        obj->confidence = detect[kScoreIndex];

        obj->type_probs.assign(static_cast<int>(base::ObjectType::MAX_OBJECT_TYPE), 0);
        obj->sub_type_probs.assign(static_cast<int>(base::ObjectSubType::MAX_OBJECT_TYPE), 0);
        obj->type_probs[static_cast<int>(obj->type)] = detect[kLabelIndex];
        obj->sub_type_probs[static_cast<int>(obj->sub_type)] = detect[kLabelIndex];

        YoloxFillBase(detect, width, height, image_width, image_height, obj);

        YoloxFillBbox3d(model_param, detect, obj);

        // TODO(lordon): add ignore truncated bool param
        YoloxTruncated(obj, image_width, image_height);

        objects->push_back(obj);
    }
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
