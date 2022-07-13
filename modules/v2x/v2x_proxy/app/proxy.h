/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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
 * @file v2x_proxy.h
 * @brief define v2x proxy class
 */
#pragma once
namespace apollo {
namespace v2x {
class Proxy {
 public:
  Proxy() = default;

  virtual ~Proxy() = default;

  bool InitFlag() { return init_flag_; }

 protected:
  bool init_flag_ = false;
};

}  // namespace v2x
}  // namespace apollo
