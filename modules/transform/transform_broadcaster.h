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

#pragma once

#include <memory>
#include <vector>

#include "cyber/cyber.h"
#include "modules/transform/proto/transform.pb.h"

namespace apollo {
namespace transform {

/** \brief This class provides an easy way to publish coordinate frame transform
 * information.
 * It will handle all the messaging and stuffing of messages.  And the function
 * prototypes lay out all the
 * necessary data needed for each message.  */

class TransformBroadcaster {
 public:
  /** \brief Constructor (needs a cyber::Node reference) */
  explicit TransformBroadcaster(const std::shared_ptr<cyber::Node>& node);

  /** \brief Send a TransformStamped message
   * The stamped data structure includes frame_id, and time, and parent_id
   * already.  */
  void SendTransform(const TransformStamped& transform);

  /** \brief Send a vector of TransformStamped messages
   * The stamped data structure includes frame_id, and time, and parent_id
   * already.  */
  void SendTransform(const std::vector<TransformStamped>& transforms);

 private:
  std::shared_ptr<cyber::Node> node_;
  std::shared_ptr<cyber::Writer<TransformStampeds>> writer_;
};
}  // namespace transform
}  // namespace apollo
