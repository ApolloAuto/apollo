// Copyright 2025 WheelOS. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//  Created Date: 2025-01-16
//  Author: daohu527

#pragma once

#include "modules/serial/common/serial_stream.h"

namespace apollo {
namespace serial {

class BaseControl {
 public:
  virtual ~BaseControl() = default;

  virtual bool Send(const ControlCommand& control_command) = 0;

  virtual Chassis GetChassis() const = 0;

  virtual bool Close() = 0;

 protected:
  std::unique_ptr<Stream> serial_stream_;
};

}  // namespace serial
}  // namespace apollo
