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

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_VISUALIZER_BASE_VISUALIZER_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_VISUALIZER_BASE_VISUALIZER_H_

// SAMPLE CODE:
//
// class MyVisualizer : public BaseVisualizer {
// public:
//     MyVisualizer() : BaseVisualizer() {}
//     virtual ~MyVisualizer() {}
//
//     virtual bool init() override {
//         // Do something.
//         return true;
//     }
//
//     virtual bool render(
//              FrameContent &content) override {
//
//          // Do something.
//          return true;
//      }
//
//      virtual std::string name() const override {
//          return "MyVisualizer";
//      }
//
// };
//
// // Register plugin.
// REGISTER_VISUALIZER(MyVisualizer);
////////////////////////////////////////////////////////
// USING CODE:
// // BaseVisualizer* visualizer = //
// BaseVisualizerRegisterer::get_instance_by_name("MyVisualizer");
// using visualizer to do somethings.
// ////////////////////////////////////////////////////

#include <string>
#include "modules/common/macro.h"
#include "modules/perception/lib/base/registerer.h"
#include "modules/perception/obstacle/camera/visualizer/frame_content.h"

namespace apollo {
namespace perception {
namespace lowcostvisualizer {

class BaseVisualizer {
 public:
    BaseVisualizer() = default;

    virtual ~BaseVisualizer() = default;

    virtual bool init() = 0;

    virtual std::string name() const = 0;

    virtual void update_camera_system(FrameContent *content) {}

    virtual void render(FrameContent *content) = 0;

    // virtual void set_motion_buffer(MotionBuffer &motion_buffer) {}

 private:
      DISALLOW_COPY_AND_ASSIGN(BaseVisualizer);
};

REGISTER_REGISTERER(BaseVisualizer);
#define REGISTER_VISUALIZER(name) REGISTER_CLASS(BaseVisualizer, name)

}  // namespace lowcostvisualizer
}  // namespace perception
}  // namespace apollo

#endif  // APOLLO_PERCEPTION_OBSTACLE_CAMERA_VISUALIZER_BASE_VISUALIZER_H_
