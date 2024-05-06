/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include <vector>

#include "Eigen/Core"

#include "modules/perception/common/base/blob.h"
#include "modules/perception/common/base/object.h"
#include "modules/perception/common/base/object_types.h"

namespace apollo {
namespace perception {
namespace camera {
/**
 * @brief Get the Objects objects from blob
 *
 * @param box3ds The input box 3ds blob
 * @param labels The input labels blob
 * @param scores The input scores blob
 * @param types The input types blob
 * @param score_threshold The threshold for score to be considered
 * @param objects The output objects blob
 */
void GetObjects(const base::BlobPtr<float> &box3ds,
                const base::BlobPtr<float> &labels,
                const base::BlobPtr<float> &scores,
                const std::vector<base::ObjectSubType> &types,
                float score_threshold, std::vector<base::ObjectPtr> *objects);

/**
 * @brief Fill the 3d bbox to object
 *
 * @param bbox
 * @param obj
 */
void FillBBox3d(const float *bbox, base::ObjectPtr obj);

/**
 * @brief Get the Subtype
 *
 * @param cls Input class label
 * @param types Output type label
 * @return base::ObjectSubType
 */
base::ObjectSubType GetSubtype(int cls,
                               const std::vector<base::ObjectSubType> &types);

}  // namespace camera
}  // namespace perception
}  // namespace apollo
