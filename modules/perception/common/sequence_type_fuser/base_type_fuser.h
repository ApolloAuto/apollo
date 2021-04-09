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
#ifndef MODULES_PERCEPTION_COMMON_SEQUENCE_TYPE_FUSER_BASE_TYPE_FUSER_H_
#define MODULES_PERCEPTION_COMMON_SEQUENCE_TYPE_FUSER_BASE_TYPE_FUSER_H_

// SAMPLE CODE:
//
// class MyTypeFuser : public BaseTypeFuser {
// public:
//     MyTypeFuser() : BaseTypeFuser() {}
//     virtual ~MyTypeFuser() {}
//
//     virtual bool Init() override {
//         // Do something.
//         return true;
//     }
//
//     virtual bool FuseType(const TypeFuserOptions& options,
//                           std::vector<std::shared_ptr<Object>>* objects)
//                           override {
//          // Do something.
//          return true;
//     }
//
//     virtual std::string name() const override {
//          return "MyTypeFuser";
//     }
//
// };
//
// // Register plugin.
// REGISTER_TYPEFUSER(MyTypeFuser);
////////////////////////////////////////////////////////
// USING CODE:
//
// BaseTypeFuser* type_fuser =
//          BaseTypeFuserRegisterer::GetInstanceByName("MyTypeFuser");
// using type_fuser to do something.

#include <memory>
#include <string>
#include <vector>

#include "modules/common/macro.h"
#include "modules/perception/lib/base/registerer.h"
#include "modules/perception/obstacle/base/object.h"

namespace apollo {
namespace perception {

struct TypeFuserOptions {
  double timestamp = 0.0;
};

class BaseTypeFuser {
 public:
  /**
   * @brief Constructor
   */
  BaseTypeFuser() {}

  /**
   * @ brief Destructor
   */
  virtual ~BaseTypeFuser() {}

  /**
   * @brief Initialize configuration
   * @return True if initialize successfully, false otherwise
   */
  virtual bool Init() = 0;

  /**
   * @brief Fuse type for each object
   * @param options Some algorithm options
   * @param objects The objects with initial object type
   * @return True if fuse type successfully, false otherwise
   */
  virtual bool FuseType(const TypeFuserOptions& options,
                        std::vector<std::shared_ptr<Object>>* objects) = 0;

  /**
   * @brief Get module name
   * @return Name of module
   */
  virtual std::string name() const = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(BaseTypeFuser);
};

REGISTER_REGISTERER(BaseTypeFuser);
#define REGISTER_TYPEFUSER(name) REGISTER_CLASS(BaseTypeFuser, name)

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_COMMON_SEQUENCE_TYPE_FUSER_BASE_TYPE_FUSER_H_
