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

#include <opencv2/opencv.hpp>

#include "modules/perception/camera_detection_multi_stage/detector/yolox3d/proto/model_param.pb.h"

#include "modules/perception/common/base/blob.h"
#include "modules/perception/common/base/object.h"

namespace apollo {
namespace perception {
namespace camera {
/**
 * @brief Return Yolox Objects
 *
 * @param objects_blob the object blob output from network
 * @param model_param The parameters of model
 * @param nms_param The parameters of NMS
 * @param width 640 resized image width
 * @param height 640 resized image height
 * @param image_width image width
 * @param image_height  the image width and height
 * @param objects pointer to the object
 */
void YoloxGetObjectsCpu(
        const std::shared_ptr<base::Blob<float>> &objects_blob,
        const yolox3d::ModelParam &model_param,
        const yolox3d::NMSParam &nms_param,
        const int width,
        const int height,
        const int image_width,
        const int image_height,
        std::vector<base::ObjectPtr> *objects);
/**
 * @brief Get all objects accoring to confidence
 *
 * @param data the output blob data
 * @param model_param The parameters of model
 * @param objects pointer to the object
 */
void YoloxGetAllObjects(
        const float *data,
        const yolox3d::ModelParam &model_param,
        const float scale,
        std::vector<std::vector<float>> *objects_out);

/**
 * @brief Get 2d bbox for objects
 *
 * @param detect pointer to the detect blob
 * @param width 640 resized image width
 * @param height 640 resized image height
 * @param image_width 1920 image width
 * @param image_height 1080 image height
 * @param obj pointer to the object
 */
void YoloxFillBase(
        const std::vector<float> &detect,
        const int width,
        const int height,
        const int image_width,
        const int image_height,
        base::ObjectPtr obj);
/**
 * @brief Add 3d bbox values for objects
 *
 * @param model_param The parameters of model
 * @param detect  output of network
 * @param obj pointer to the object
 */
void YoloxFillBbox3d(const yolox3d::ModelParam &model_param, const std::vector<float> &detect, base::ObjectPtr obj);

/**
 * @brief Computes IoU between bboxes.
 * @param box1 object label
 * @param box2 object label
 * @return Returns the IoU of two bboxes
 */
float YoloxBboxIOU(const std::vector<float> &box1, const std::vector<float> &box2);
/**
 * @brief object is truncated or not
 *
 * @param obj pointer to the object
 * @param image_width 1920 image width
 * @param image_height 1080 image height
 * @return true
 * @return false
 */
void YoloxTruncated(base::ObjectPtr obj, const int image_width, const int image_height);

/**
 * @brief Clamp target value between low and high tools for iou
 * @param val target value
 * @param low min value
 * @param high max value
 * @return Returns clamped value
 */
template <typename T>
constexpr T Yoloxclamp(const T &val, const T &low, const T &high) {
    return val < low ? low : (high < val ? high : val);
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
