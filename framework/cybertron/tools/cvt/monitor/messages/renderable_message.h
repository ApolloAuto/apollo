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

#ifndef RENDERABLEMESSAGE_H
#define RENDERABLEMESSAGE_H

#include <string>

namespace adu {
namespace common {

class Point3D;
namespace header {
class Header;
}

namespace sensor {
class NovatelHeader;
class WheelMeasurement;
}

}  // common
}  // adu

class Screen;

class RenderableMessage {
 public:
  explicit RenderableMessage(RenderableMessage* parent = nullptr)
      : parent_(parent) {}

  virtual ~RenderableMessage() { parent_ = nullptr; }

  virtual void Render(const Screen* s, int key) = 0;

  RenderableMessage* parent(void) const { return parent_; }
  void set_parent(RenderableMessage* parent) {
    if (parent == parent_) {
      return;
    }
    parent_ = parent;
  }

  virtual RenderableMessage* Child(int /* lineNo */) const { return nullptr; }

 protected:
  RenderableMessage* parent_;
};

#endif  // RENDERABLEMESSAGE_H
