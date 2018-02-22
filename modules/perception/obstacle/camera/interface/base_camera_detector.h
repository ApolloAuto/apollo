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

// The base class of camera 2D object detection

#ifndef ADU_PERCEPTION_OBSTACLE_CAMERA_INTERFACE_BASE_CAMERA_DETECTOR_H
#define ADU_PERCEPTION_OBSTACLE_CAMERA_INTERFACE_BASE_CAMERA_DETECTOR_H

// SAMPLE CODE:
//
// class DefaultCameraDetector : public BaseCameraDetector {
// public:
//     DefaultCameraDetector() : BaseCameraDetector() {}
//     virtual ~DefaultCameraDetector() {}
//
//     virtual bool init(const CameraDetectorInitOptions& options) override {
//         // Do something.
//         return true;
//     }
//
//     virtual bool detect(const cv::Mat& frame,
//              const CameraDetectorOptions& options,
//              std::vector<VisualObjectPtr>* objects) override {
//          // Do something.
//          return true;
//      }
//
//      virtual std::string name() const override {
//          return "DefaultCameraDetector";
//      }
//
// };
//
// // Register plugin.
// REGISTER_CAMERA_DETECTOR(DefaultCameraDetector);
////////////////////////////////////////////////////////
// USING CODE:
//
// BaseCameraDetector* camera_detector =
//    BaseCameraDetectorRegisterer::get_instance_by_name("DefaultCameraDetector");
// using camera_detector to do somethings.
// ////////////////////////////////////////////////////

#include <string>
#include <vector>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include "lib/base/noncopyable.h"
#include "lib/base/registerer.h"
#include "obstacle/camera/common/visual_object.h"
#include "obstacle/common/camera.h"

namespace adu {
namespace perception {
namespace obstacle {

struct CameraDetectorInitOptions {
    std::shared_ptr<CameraDistortD> intrinsic;
};

struct CameraDetectorOptions {
    cv::Mat gray_frame;
    cv::Mat range_frame;
    std::shared_ptr<CameraDistortD> intrinsic;
    std::shared_ptr<Eigen::Matrix4d> extrinsic_ground2camera;
    std::shared_ptr<Eigen::Matrix4d> extrinsic_stereo;
};

class BaseCameraDetector {
public:
    BaseCameraDetector() = default;
    virtual ~BaseCameraDetector() = default;

    virtual bool init(const CameraDetectorInitOptions& options
            = CameraDetectorInitOptions()) = 0;

    // @brief: Frame from camera -> objects.
    // @param [in]: raw frame from camera.
    // @param [in]: options.
    // @param [out]: detected objects.
    virtual bool detect(const cv::Mat& frame,
            const CameraDetectorOptions& options,
            std::vector<VisualObjectPtr>* objects) = 0;
    virtual bool multitask(const cv::Mat& frame,
                        const CameraDetectorOptions& options,
                        std::vector<VisualObjectPtr>* objects, cv::Mat *mask){
        return false;
    };
    // @brief: Extract feature for each object.
    // @param [in/out]: objects with 2D bbox.
    virtual bool extract(std::vector<VisualObjectPtr>* objects) {
        return true;
    }

    virtual std::string name() const = 0;

private:
    DISALLOW_COPY_AND_ASSIGN(BaseCameraDetector);
};

REGISTER_REGISTERER(BaseCameraDetector);
#define REGISTER_CAMERA_DETECTOR(name) REGISTER_CLASS(BaseCameraDetector, name)

}  // namespace obstacle
}  // namespace perception
}  // namespace adu

#endif  // ADU_PERCEPTION_OBSTACLE_CAMERA_INTERFACE_BASE_CAMERA_DETECTOR_H
