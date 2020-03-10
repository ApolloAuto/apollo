/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
 * @brief story telling container
 */

#pragma once

#include <memory>
#include <string>

#include "modules/prediction/common/prediction_map.h"
#include "modules/prediction/container/container.h"
#include "modules/storytelling/proto/story.pb.h"

namespace apollo {
namespace prediction {

class StoryTellingContainer : public Container {
 public:
  /**
   * @brief Constructor
   */
  StoryTellingContainer() = default;

  /**
   * @brief Destructor
   */
  virtual ~StoryTellingContainer() = default;

  /**
   * @brief Insert a data message into the container
   * @param Data message to be inserted in protobuf
   */
  void Insert(const ::google::protobuf::Message& message) override;

  /**
   * @brief Get ADC junction
   * @return A pointer to ADC junction information
   */
  std::shared_ptr<const hdmap::JunctionInfo> ADCJunction() const;

  /**
   * @brief Get ADC junction id
   * @return A reference of ADC_junction_id
   */
  const std::string& ADCJunctionId() const;

  /**
   * @brief Compute ADC's distance to junction
   * @return ADC's distance to junction
   */
  double ADCDistanceToJunction() const;

 private:
  apollo::storytelling::CloseToJunction close_to_junction_;
};

}  // namespace prediction
}  // namespace apollo
