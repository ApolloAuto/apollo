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

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_DUMMY_DUMMY_ALGORITHMS_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_DUMMY_DUMMY_ALGORITHMS_H_

#include <memory>
#include <string>
#include <vector>

#include "modules/perception/obstacle/camera/common/projector.h"
#include "modules/perception/obstacle/camera/interface/base_camera_detector.h"
#include "modules/perception/obstacle/camera/interface/base_camera_tracker.h"
#include "modules/perception/obstacle/camera/interface/base_camera_transformer.h"

namespace apollo {
namespace perception {

class DummyCameraDetector : public BaseCameraDetector {
 public:
  DummyCameraDetector() : BaseCameraDetector() {}
  virtual ~DummyCameraDetector() {}

  bool Init(const CameraDetectorInitOptions& options =
                CameraDetectorInitOptions()) override {
    return true;
  }

  bool Detect(const cv::Mat& frame, const CameraDetectorOptions& options,
              std::vector<std::shared_ptr<VisualObject>>* objects) override {
    return true;
  }

  bool Multitask(const cv::Mat& frame, const CameraDetectorOptions& options,
                 std::vector<std::shared_ptr<VisualObject>>* objects,
                 cv::Mat* mask) override {
    return true;
  }

  bool Extract(std::vector<std::shared_ptr<VisualObject>>* objects) override {
    return true;
  }

  std::string Name() const override { return "DummyCameraDetector"; }

 private:
  DISALLOW_COPY_AND_ASSIGN(DummyCameraDetector);
};

class DummyCameraTracker : public BaseCameraTracker {
 public:
  DummyCameraTracker() : BaseCameraTracker() {}
  virtual ~DummyCameraTracker() {}

  bool Init() override { return true; }

  /*
  bool predict_velocity(
      const cv::Mat &frame, const std::vector<std::shared_ptr<VisualObject>>
  &objects,
      double timestamp, const CameraTrackerOptions &options,
      std::vector<std::shared_ptr<VisualObject>> *tracked_objects) override {
    *tracked_objects = objects;
    this->trans_object_to_world(options, tracked_objects);
    return true;
  }

  bool predict_shape(
      const cv::Mat &frame, const std::vector<std::shared_ptr<VisualObject>>
  &objects,
      double timestamp, const CameraTrackerOptions &options,
      std::vector<std::shared_ptr<VisualObject>> *tracked_objects) override {
    return true;
  }
  */

  bool Associate(const cv::Mat& img, const double& timestamp,
                 std::vector<std::shared_ptr<VisualObject>>* objects) override {
    return true;
  }

  std::string Name() const override { return "DummyCameraTracker"; }

 private:
  DISALLOW_COPY_AND_ASSIGN(DummyCameraTracker);
};

class DummyCameraTransformer : public BaseCameraTransformer {
 public:
  DummyCameraTransformer() : BaseCameraTransformer() {}
  virtual ~DummyCameraTransformer() {}

  bool Init() override { return true; }

  bool Transform(std::vector<std::shared_ptr<VisualObject>>* objects) override {
    return true;
  }

  bool SetExtrinsics(const Eigen::Matrix<double, 4, 4>& extrinsics) override {
    return true;
  }

  std::string Name() const override { return "DummyCameraTransformer"; }

 private:
  DISALLOW_COPY_AND_ASSIGN(DummyCameraTransformer);
};

class DummyProjector : public BaseProjector {
 public:
  bool project(std::vector<float>* feature) override { return true; }
};

REGISTER_CAMERA_DETECTOR(DummyCameraDetector);
REGISTER_CAMERA_TRACKER(DummyCameraTracker);
REGISTER_CAMERA_TRANSFORMER(DummyCameraTransformer);

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_DUMMY_DUMMY_ALGORITHMS_H_
