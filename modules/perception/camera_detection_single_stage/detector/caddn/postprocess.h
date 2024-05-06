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

#include <memory>
#include <vector>

#include "Eigen/Core"

#include "modules/perception/camera_detection_single_stage/detector/caddn/proto/model_param.pb.h"

#include "modules/perception/common/base/blob.h"
#include "modules/perception/common/base/object.h"
#include "modules/perception/common/base/object_types.h"

namespace apollo {
namespace perception {
namespace camera {

/**
 * @brief Get the Caddn Objects objects from Blob
 *
 * @param objects The objects container
 * @param model_param The Caddn model param
 * @param types The object types
 * @param boxes The boxes container
 * @param labels The labels container
 * @param scores The scores container
 */
void GetCaddnObjects(std::vector<base::ObjectPtr> *objects,
                     const caddn::ModelParam &model_param,
                     const std::vector<base::ObjectSubType> &types,
                     const base::BlobPtr<float> &boxes,
                     const base::BlobPtr<float> &labels,
                     const base::BlobPtr<float> &scores);
/**
 * @brief Get the Caddn Objects objects from Blob
 *
 * @param V2C  the object container
 * @param R  the rotation matrix
 * @param bbox_lidar  lidar boxes
 * @param bbox_camera camera boxes
 */
void Bbox3dLidar2Camera(const Eigen::Matrix<float, 3, 4> &V2C,
                        const Eigen::Matrix<float, 3, 3> &R,
                        const float *bbox_lidar,
                        std::vector<float> *bbox_camera);
/**
 * @brief Get the Subtype objects from Blob
 *
 * @param cls  the object container
 * @param types  the object types
 * @return base::ObjectSubType
 */
base::ObjectSubType GetSubtype(int cls,
                               const std::vector<base::ObjectSubType> &types);
/**
 * @brief Add objects values to object
 *
 * @param obj object
 * @param bbox  bbox result
 */
void FillCaddnBbox3d(base::ObjectPtr obj, const float *bbox);

}  // namespace camera
}  // namespace perception
}  // namespace apollo
