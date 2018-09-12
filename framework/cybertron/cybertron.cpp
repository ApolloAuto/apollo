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

#include "cybertron/cybertron.h"

namespace apollo {
namespace cybertron {

std::unique_ptr<Node> CreateNode(const std::string& node_name,
                                 const std::string& name_space) {
  if (!OK()) {
    // TODO @nizhongjun
    // add some hint log
    AERROR << "cybertron has not inited.";
    return nullptr;
  }
  std::unique_ptr<Node> node(new Node(node_name, name_space));
  return std::move(node);
}

}  // namespace cybertron
}  // namespace apollo