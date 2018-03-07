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

#include "modules/perception/obstacle/camera/common/projector.h"
#include "modules/perception/obstacle/camera/interface/base_camera_detector.h"
#include "modules/perception/obstacle/camera/interface/base_camera_tracker.h"
#include "modules/perception/obstacle/camera/interface/base_camera_transformer.h"
//#include "modules/perception/obstacle/camera/interface/base_camera_parser.h"

namespace apollo {
namespace perception {

/*
class DummyCameraParser : public BaseCameraParser {
 public:
  DummyCameraParser() : BaseCameraParser() {}
  virtual ~DummyCameraParser() {}

  virtual bool init() override { return true; }

  virtual bool parse(const cv::Mat &frame, const CameraParserOptions &options,
                     cv::Mat *result) override {
    return true;
  }

  virtual std::string name() const override { return "DummyCameraParser"; }

 private:
  DISALLOW_COPY_AND_ASSIGN(DummyCameraParser);
};
*/

class DummyCameraDetector : public BaseCameraDetector {
 public:
  DummyCameraDetector() : BaseCameraDetector() {}
  virtual ~DummyCameraDetector() {}

  virtual bool Init(const CameraDetectorInitOptions &options =
                    CameraDetectorInitOptions()) override {
    return true;
  }

  virtual bool Detect(const cv::Mat &frame,
                      const CameraDetectorOptions &options,
                      std::vector<VisualObjectPtr> *objects) override {
    return true;
  }

  virtual bool Extract(std::vector<VisualObjectPtr>* objects) override {
    return true;
  }

  virtual std::string Name() const override { return "DummyCameraDetector"; }

 private:
  DISALLOW_COPY_AND_ASSIGN(DummyCameraDetector);
};

class DummyCameraTracker : public BaseCameraTracker {
 public:
  DummyCameraTracker() : BaseCameraTracker() {}
  virtual ~DummyCameraTracker() {}

  virtual bool Init() override { return true; }

  /*
  virtual bool predict_velocity(
      const cv::Mat &frame, const std::vector<VisualObjectPtr> &objects,
      double timestamp, const CameraTrackerOptions &options,
      std::vector<VisualObjectPtr> *tracked_objects) override {
    *tracked_objects = objects;
    this->trans_object_to_world(options, tracked_objects);
    return true;
  }

  virtual bool predict_shape(
      const cv::Mat &frame, const std::vector<VisualObjectPtr> &objects,
      double timestamp, const CameraTrackerOptions &options,
      std::vector<VisualObjectPtr> *tracked_objects) override {
    return true;
  }
  */

  virtual bool Associate(const cv::Mat& img, const float& timestamp,
                         std::vector<VisualObjectPtr>* objects) override {
    return true;
  }

  virtual std::string Name() const override { return "DummyCameraTracker"; }

 private:
  DISALLOW_COPY_AND_ASSIGN(DummyCameraTracker);
};

class DummyCameraTransformer : public BaseCameraTransformer {
 public:
  DummyCameraTransformer() : BaseCameraTransformer() {}
  virtual ~DummyCameraTransformer() {}

  virtual bool Init() override { return true; }

  virtual bool Transform(std::vector<VisualObjectPtr>* objects) override {
    return true;
  }

  virtual std::string Name() const override {
    return "DummyCameraTransformer";
  }

 private:
  DISALLOW_COPY_AND_ASSIGN(DummyCameraTransformer);
};

class DummyProjector : public BaseProjector {
 public:
  virtual void project(std::vector<float> &feature) override {}
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_DUMMY_DUMMY_ALGORITHMS_H_
