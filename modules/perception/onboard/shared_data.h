/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#ifndef MODULES_PERCEPTION_ONBOARD_SHARED_DATA_H_
#define MODULES_PERCEPTION_ONBOARD_SHARED_DATA_H_

#include <string>

#include "modules/common/log.h"
#include "modules/common/macro.h"
#include "modules/perception/lib/base/registerer.h"

namespace apollo {
namespace perception {

class SharedData {
 public:
  SharedData() {}
  virtual ~SharedData() {}

  virtual bool Init() = 0;

  // this api should clear all the memory used,
  // and would be called by SharedDataManager when reset DAGStreaming.
  virtual void Reset() { CHECK(false) << "reset() not implemented."; }

  virtual void RemoveStaleData() {
    CHECK(false) << "remove_stale_data() not implemented.";
  }

  virtual std::string name() const = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(SharedData);
};

REGISTER_REGISTERER(SharedData);

#define REGISTER_SHAREDDATA(name) REGISTER_CLASS(SharedData, name)

}  // namespace perception
}  // namespace apollo
#endif  // MODULES_PERCEPTION_ONBOARD_SHARED_DATA_H_
