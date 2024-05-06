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

#include <map>

#include "modules/perception/common/base/object.h"
#include "modules/perception/common/base/object_pool_types.h"

namespace apollo {
namespace perception {
namespace camera {

class ObjectMaintainer {
 public:
  ObjectMaintainer() = default;
  ~ObjectMaintainer() = default;
  /**
   * @brief Maintain a collection of objects that can be added based on the
   * index. By comparing the Subtyping probability of the new object and the
   * Subtyping probability of the new and old objects, decide whether to update
   * the old object. If the Subtyping probability of the new object is greater
   * than the Subtyping probability of the old object, the old object is updated
   * to the new object
   *
   * @param idx The index to maintain in the collection
   * @param obj New object to maintain in the collection
   * @return status if the object was updated successfully, false otherwise
   */
  bool Add(int idx, base::ObjectPtr obj);

 protected:
  std::map<int, base::ObjectPtr> assigned_index_;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
