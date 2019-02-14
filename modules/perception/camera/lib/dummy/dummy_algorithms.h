/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <map>
#include <string>

#include "modules/perception/camera/lib/interface/base_calibration_service.h"
#include "modules/perception/camera/lib/interface/base_calibrator.h"
#include "modules/perception/camera/lib/interface/base_feature_extractor.h"
#include "modules/perception/camera/lib/interface/base_inference_engine.h"
#include "modules/perception/camera/lib/interface/base_landmark_detector.h"
#include "modules/perception/camera/lib/interface/base_lane_detector.h"
#include "modules/perception/camera/lib/interface/base_lane_postprocessor.h"
#include "modules/perception/camera/lib/interface/base_lane_tracker.h"
#include "modules/perception/camera/lib/interface/base_obstacle_detector.h"
#include "modules/perception/camera/lib/interface/base_obstacle_postprocessor.h"
#include "modules/perception/camera/lib/interface/base_obstacle_tracker.h"
#include "modules/perception/camera/lib/interface/base_obstacle_transformer.h"
#include "modules/perception/camera/lib/interface/base_scene_parser.h"
#include "modules/perception/camera/lib/interface/base_traffic_light_detector.h"
#include "modules/perception/camera/lib/interface/base_traffic_light_tracker.h"

namespace apollo {
namespace perception {
namespace camera {

class DummyInferenceEngine : public BaseInferenceEngine {
 public:
  DummyInferenceEngine() : BaseInferenceEngine() {}

  virtual ~DummyInferenceEngine() {}

  bool Init(const InferenceEngineInitOptions &options =
                InferenceEngineInitOptions()) override {
    return true;
  }

  bool Infer(const InferenceEngineOptions &options,
             CameraFrame *frame) override {
    return true;
  }

  std::string Name() const override { return "DummyInferenceEngine"; }
};

class DummyLaneDetector : public BaseLaneDetector {
 public:
  DummyLaneDetector() : BaseLaneDetector() {}

  virtual ~DummyLaneDetector() {}

  bool Init(const LaneDetectorInitOptions &options = {}) override {
    return true;
  }

  bool Detect(const LaneDetectorOptions &options, CameraFrame *frame) override {
    return true;
  }

  std::string Name() const override { return "DummyLaneDetector"; }
};

class DummyLanePostprocessor : public BaseLanePostprocessor {
 public:
  DummyLanePostprocessor() : BaseLanePostprocessor() {}

  virtual ~DummyLanePostprocessor() {}

  bool Init(const LanePostprocessorInitOptions &options =
                LanePostprocessorInitOptions()) override {
    return true;
  }

  bool Process2D(const LanePostprocessorOptions &options,
                 CameraFrame *frame) override {
    return true;
  }

  bool Process3D(const LanePostprocessorOptions &options,
                 CameraFrame *frame) override {
    return true;
  }

  std::string Name() const override { return "DummyLanePostprocessor"; }
};

class DummyLaneTracker : public BaseLaneTracker {
 public:
  DummyLaneTracker() : BaseLaneTracker() {}

  virtual ~DummyLaneTracker() {}

  bool Init(const LaneTrackerInitOptions &options =
                LaneTrackerInitOptions()) override {
    return true;
  }

  bool Track(const LaneTrackerOptions &options, CameraFrame *frame) override {
    return true;
  }

  std::string Name() const override { return "DummyLaneTracker"; }
};

class DummyObstacleDetector : public BaseObstacleDetector {
 public:
  DummyObstacleDetector() : BaseObstacleDetector() {}

  virtual ~DummyObstacleDetector() {}

  bool Init(const ObstacleDetectorInitOptions &options =
                ObstacleDetectorInitOptions()) override {
    return true;
  }

  bool Detect(const ObstacleDetectorOptions &options,
              CameraFrame *frame) override {
    return true;
  }

  std::string Name() const override { return "DummyObstacleDetector"; }
};

class DummyObstaclePostprocessor : public BaseObstaclePostprocessor {
 public:
  DummyObstaclePostprocessor() : BaseObstaclePostprocessor() {}

  virtual ~DummyObstaclePostprocessor() {}

  bool Init(const ObstaclePostprocessorInitOptions &options =
                ObstaclePostprocessorInitOptions()) override {
    return true;
  }

  bool Process(const ObstaclePostprocessorOptions &options,
               CameraFrame *frame) override {
    return true;
  }

  std::string Name() const override { return "DummyObstaclePostprocessor"; }
};

class DummyObstacleTracker : public BaseObstacleTracker {
 public:
  bool Init(const ObstacleTrackerInitOptions &options) override { return true; }

  bool Predict(const ObstacleTrackerOptions &options,
               CameraFrame *frame) override {
    return true;
  }

  bool Associate2D(const ObstacleTrackerOptions &options,
                   CameraFrame *frame) override {
    return true;
  }

  bool Associate3D(const ObstacleTrackerOptions &options,
                   CameraFrame *frame) override {
    return true;
  }

  bool Track(const ObstacleTrackerOptions &options,
             CameraFrame *frame) override {
    frame->tracked_objects.clear();
    for (auto &detected_object : frame->detected_objects) {
      frame->tracked_objects.push_back(detected_object);
    }
    return true;
  }

  std::string Name() const override { return "DummyObstacleTracker"; }
};

class DummyObstacleTransformer : public BaseObstacleTransformer {
 public:
  bool Init(const ObstacleTransformerInitOptions &options) override {
    return true;
  }
  bool Transform(const ObstacleTransformerOptions &options,
                 CameraFrame *frame) override {
    return true;
  }
  std::string Name() const override { return "DummyObstacleTransformer"; }
};

class DummyFeatureExtractor : public BaseFeatureExtractor {
 public:
  DummyFeatureExtractor() : BaseFeatureExtractor() {}
  virtual ~DummyFeatureExtractor() {}
  bool Init(const FeatureExtractorInitOptions &init_options) override {
    return true;
  }

  bool Extract(const FeatureExtractorOptions &options,
               CameraFrame *frame) override {
    return true;
  }
  std::string Name() const override { return "DummyFeatureExtractor"; }
};

class DummyLandmarkDetector : public BaseLandmarkDetector {
 public:
  DummyLandmarkDetector() : BaseLandmarkDetector() {}
  virtual ~DummyLandmarkDetector() {}
  bool Init(const LandmarkDetectorInitOptions &options =
                LandmarkDetectorInitOptions()) override {
    return true;
  }

  bool Detect(const LandmarkDetectorOptions &options,
              CameraFrame *frame) override {
    return true;
  }
  std::string Name() const override { return "DummyLandmarkDetector"; }
};

class DummyCalibrator : public BaseCalibrator {
 public:
  DummyCalibrator() : BaseCalibrator() {}
  virtual ~DummyCalibrator() {}
  bool Init(
      const CalibratorInitOptions &options = CalibratorInitOptions()) override {
    return true;
  }
  bool Calibrate(const CalibratorOptions &options,
                 float *pitch_angle) override {
    return true;
  }
  std::string Name() const override { return "DummyCalibrator"; }
};

class DummyCalibrationService : public BaseCalibrationService {
 public:
  DummyCalibrationService() : BaseCalibrationService() {}
  virtual ~DummyCalibrationService() {}
  bool Init(const CalibrationServiceInitOptions &options =
                CalibrationServiceInitOptions()) override {
    return true;
  }
  bool BuildIndex() override { return true; }
  bool QueryDepthOnGroundPlane(int x, int y, double *depth) const override {
    return false;
  }
  bool QueryPoint3dOnGroundPlane(int x, int y,
                                 Eigen::Vector3d *point3d) const override {
    return false;
  }
  bool QueryGroundPlaneInCameraFrame(
      Eigen::Vector4d *plane_param) const override {
    return false;
  }
  bool QueryCameraToGroundHeightAndPitchAngle(float *height,
                                              float *pitch) const override {
    return false;
  }
  float QueryCameraToGroundHeight() { return 0.f; }
  float QueryPitchAngle() { return 0.f; }
  void Update(CameraFrame *frame) override {
    // do nothing
  }

  void SetCameraHeightAndPitch(
      const std::map<std::string, float> &name_camera_ground_height_map,
      const std::map<std::string, float> &name_camera_pitch_angle_diff_map,
      const float &pitch_angle_master_sensor) override {
    // do nothing
  }

  std::string Name() const override { return "DummyCalibrationService"; }
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
