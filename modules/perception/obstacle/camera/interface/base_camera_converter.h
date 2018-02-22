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

// The base class of converting 2D detections into 3D objects


#ifndef ADU_PERCEPTION_OBSTACLE_CAMERA_INTERFACE_BASE_CAMERA_TRANSFORMER_H
#define ADU_PERCEPTION_OBSTACLE_CAMERA_INTERFACE_BASE_CAMERA_TRANSFORMER_H

// SAMPLE CODE:
//
// class DefaultCameraTransformer : public BaseCameraTransformer {
// public:
//     DefaultCameraTransformer() : BaseCameraTransformer() {}
//     virtual ~DefaultCameraTransformer() {}
//
//     virtual bool init() override {
//         // Do something.
//         return true;
//     }
//
//     virtual bool transform(const cv::Mat& frame,
//              const CameraTransformerOptions& options,
//              std::vector<VisualObjectPtr>* objects) override {
//          // Do something.
//          return true;
//      }
//
//      virtual std::string name() const override {
//          return "DefaultCameraTransformer";
//      }
//
// };
//
// // Register plugin.
// REGISTER_CAMERA_TRANSFORMER(DefaultCameraTransformer);
////////////////////////////////////////////////////////
// USING CODE:
//
// BaseCameraTransformer* camera_transformer =
//    BaseCameraTransformerRegisterer::get_instance_by_name("DefaultCameraTransformer");
// using camera_transformer to do somethings.
// ////////////////////////////////////////////////////

#include <string>
#include <vector>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include "lib/base/noncopyable.h"
#include "lib/base/registerer.h"
#include "obstacle/camera/common/visual_object.h"

namespace adu {
namespace perception {
namespace obstacle {

struct CameraTransformerOptions {
};

class BaseCameraTransformer {
public:
    BaseCameraTransformer() = default;
    virtual ~BaseCameraTransformer() = default;

    virtual bool init() = 0;

    // @brief: Frame from camera  -> objects.
    // @param [in]: raw frame from camera.
    // @param [in]: options.
    // @param [out]: transformed objects.
    virtual bool transform(const cv::Mat& frame,
            const CameraTransformerOptions& options,
            std::vector<VisualObjectPtr>* objects) = 0;

    virtual std::string name() const = 0;

private:
    DISALLOW_COPY_AND_ASSIGN(BaseCameraTransformer);
};

REGISTER_REGISTERER(BaseCameraTransformer);
#define REGISTER_CAMERA_TRANSFORMER(name) REGISTER_CLASS(BaseCameraTransformer, name)

}  // namespace obstacle
}  // namespace perception
}  // namespace adu

#endif  // ADU_PERCEPTION_OBSTACLE_CAMERA_INTERFACE_BASE_CAMERA_TRANSFORMER_H
