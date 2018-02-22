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

/**
 * @file
 */

#ifndef MODULES_PREDICTION_CONTAINER_RELATIVE_MAP_CONTAINER_H_
#define MODULES_PREDICTION_CONTAINER_RELATIVE_MAP_CONTAINER_H_

#include "modules/prediction/container/container.h"
#include "modules/map/relative_map/proto/navigation.pb.h"

namespace apollo {
namespace prediction {

class RelativeMapContainer : public Container {
 public:
  /**
   * @brief Constructor
   */
  RelativeMapContainer() = default;

  /**
   * @brief Destructor
   */
  virtual ~RelativeMapContainer() = default;

  /**
   * @brief Insert a data message into the container
   * @param Data message to be inserted in protobuf
   */
  void Insert(const ::google::protobuf::Message& message) override;

 private:
  apollo::relative_map::MapMsg relative_map_;
};

}  // namespace prediction
}  // namespace apollo

#endif  // MODULES_PREDICTION_CONTAINER_RELATIVE_MAP_CONTAINER_H_
