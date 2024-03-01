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

#ifndef CYBER_DATA_FUSION_DATA_FUSION_H_
#define CYBER_DATA_FUSION_DATA_FUSION_H_

#include <deque>
#include <memory>
#include <string>
#include <type_traits>
#include <typeinfo>
#include <vector>

#include "cyber/common/types.h"

namespace apollo {
namespace cyber {
namespace data {
namespace fusion {

template <typename ...M>
class DataFusion {
 public:
  virtual ~DataFusion() {}
  virtual bool Fusion(uint64_t* index, std::shared_ptr<M>& ...m) = 0;              // NOLINT
};

}  // namespace fusion
}  // namespace data
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_DATA_FUSION_DATA_FUSION_H_
