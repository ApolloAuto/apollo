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
#include <memory>
#include <set>
#include <vector>

#include "modules/perception/camera_detection_2d/detector/yolov3/proto/model_param.pb.h"

#include "modules/perception/common/base/blob.h"
#include "modules/perception/common/base/object.h"

namespace apollo {
namespace perception {
namespace camera {
/**
 * @brief Return Yolov 3 Objects
 *
 * @param objects_blob the object blob output from network
 * @param model_param The parameters of model
 * @param nms_param The parameters of NMS
 * @param width 416 resized image width
 * @param height 416 resized image height
 * @param image_width image width
 * @param image_height  the image width and height
 * @param objects pointer to the object
 */
void GetYolov3ObjectsCpu(const std::shared_ptr<base::Blob<float>> &objects_blob,
                         const yolov3::ModelParam &model_param,
                         const yolov3::NMSParam &nms_param, const int width,
                         const int height, const int image_width,
                         const int image_height,
                         std::vector<base::ObjectPtr> *objects);
/**
 * @brief Get all objects accoring to confidence
 *
 * @param data the output blob data
 * @param model_param The parameters of model
 * @param objects pointer to the object
 */
void GetAllObjects(const float *data, const yolov3::ModelParam &model_param,
                   std::vector<std::vector<float>> *objects);
/**
 * @brief Get all objects accoring to confidence
 *
 * @param detect_objects  the output blob data
 * @param nms_threshold   the threshold of NMS
 * @param detect_nms  the output blob data
 */
void ApplyNms(const std::vector<std::vector<float>> &detect_objects,
              const float nms_threshold,
              std::vector<std::vector<float>> *detect_nms);
/**
 * @brief Get 2d bbox for objects
 *
 * @param detect pointer to the detect blob
 * @param width 416 resized image width
 * @param height 416 resized image height
 * @param image_width 1920 image width
 * @param image_height 1080 image height
 * @param obj pointer to the object
 */
void FillBase(const std::vector<float> &detect, const int width,
              const int height, const int image_width, const int image_height,
              base::ObjectPtr obj);
/**
 * @brief Add 3d bbox values for objects
 *
 * @param with_box3d use 3d result
 * @param detect  output of network
 * @param obj pointer to the object
 */
void FillBbox3d(bool with_box3d, const std::vector<float> &detect,
                base::ObjectPtr obj);
/**
 * @brief NMS for objects
 *
 * @param detect_objects  the output result after GetAllObjects
 * @param nms_threshold   the threshold of NMS
 * @param detect_nms  output result
 */
void NmsForObjects(const std::vector<std::vector<float>> &detect_objects,
                   const float nms_threshold,
                   std::vector<std::vector<float>> *detect_nms);
/**
 * @brief Computes IoU between bboxes.
 * @param box1 object label
 * @param box2 object label
 * @return Returns the IoU of two bboxes
 */
float BboxIOU(const std::vector<float> &box1, const std::vector<float> &box2);

/**
 * @brief Clamp target value between low and high tools for iou
 * @param val target value
 * @param low min value
 * @param high max value
 * @return Returns clamped value
 */
template <typename T>
constexpr T clamp(const T &val, const T &low, const T &high) {
  return val < low ? low : (high < val ? high : val);
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
