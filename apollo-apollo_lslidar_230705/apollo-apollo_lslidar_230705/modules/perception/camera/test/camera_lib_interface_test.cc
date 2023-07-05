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
#include "gtest/gtest.h"

#include "modules/perception/camera/lib/interface/base_calibration_service.h"
#include "modules/perception/camera/lib/interface/base_calibrator.h"
#include "modules/perception/camera/lib/interface/base_camera_perception.h"
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

class MyInferenceEngine : public BaseInferenceEngine {
 public:
  MyInferenceEngine() : BaseInferenceEngine() {}
  virtual ~MyInferenceEngine() {}

  bool Init(const InferenceEngineInitOptions& options =
                InferenceEngineInitOptions()) override {
    return true;
  }

  bool Infer(const InferenceEngineOptions& options,
             CameraFrame* frame) override {
    return true;
  }

  std::string Name() const override { return "MyInferenceEngine"; }
};

REGISTER_INFERENCE_ENGINE(MyInferenceEngine);

TEST(ObstacleInterfaceTest, test_inference_engine) {
  BaseInferenceEngine* inference_engine =
      BaseInferenceEngineRegisterer::GetInstanceByName("MyInferenceEngine");
  EXPECT_NE(inference_engine, nullptr);
  EXPECT_EQ(inference_engine->Name(), "MyInferenceEngine");
}

class MyObstacleDetector : public BaseObstacleDetector {
 public:
  MyObstacleDetector() : BaseObstacleDetector() {}
  virtual ~MyObstacleDetector() {}

  bool Init(const ObstacleDetectorInitOptions& options =
                ObstacleDetectorInitOptions()) override {
    return true;
  }

  bool Detect(const ObstacleDetectorOptions& options,
              CameraFrame* frame) override {
    return true;
  }

  std::string Name() const override { return "MyObstacleDetector"; }
};

REGISTER_OBSTACLE_DETECTOR(MyObstacleDetector);

TEST(ObstacleInterfaceTest, test_camera_detector) {
  BaseObstacleDetector* camera_detector =
      BaseObstacleDetectorRegisterer::GetInstanceByName("MyObstacleDetector");
  EXPECT_NE(camera_detector, nullptr);
  EXPECT_EQ(camera_detector->Name(), "MyObstacleDetector");
}

class MyObstacleTracker : public BaseObstacleTracker {
 public:
  MyObstacleTracker() : BaseObstacleTracker() {}
  virtual ~MyObstacleTracker() {}

  bool Init(const ObstacleTrackerInitOptions& options) override { return true; }

  bool Predict(const ObstacleTrackerOptions& options,
               CameraFrame* frame) override {
    return true;
  }

  bool Associate2D(const ObstacleTrackerOptions& options,
                   CameraFrame* frame) override {
    return true;
  }

  bool Associate3D(const ObstacleTrackerOptions& options,
                   CameraFrame* frame) override {
    return true;
  }

  bool Track(const ObstacleTrackerOptions& options,
             CameraFrame* frame) override {
    return true;
  }

  std::string Name() const override { return "MyObstacleTracker"; }
};  // class MyObstacleTracker

REGISTER_OBSTACLE_TRACKER(MyObstacleTracker);

TEST(ObstacleInterfaceTest, test_camera_tracker) {
  BaseObstacleTracker* camera_tracker =
      BaseObstacleTrackerRegisterer::GetInstanceByName("MyObstacleTracker");
  EXPECT_NE(camera_tracker, nullptr);
  EXPECT_EQ(camera_tracker->Name(), "MyObstacleTracker");
}

class MyObstacleTransformer : public BaseObstacleTransformer {
 public:
  MyObstacleTransformer() {}
  virtual ~MyObstacleTransformer() {}

  bool Init(const ObstacleTransformerInitOptions& options) override {
    return true;
  }

  bool Transform(const ObstacleTransformerOptions& options,
                 CameraFrame* frame) override {
    return true;
  }

  std::string Name() const override { return "MyObstacleTransformer"; }
};  // class MyObstacleTransformer

REGISTER_OBSTACLE_TRANSFORMER(MyObstacleTransformer);

TEST(ObstacleInterfaceTest, test_camera_transformer) {
  BaseObstacleTransformer* camera_transformer =
      BaseObstacleTransformerRegisterer::GetInstanceByName(
          "MyObstacleTransformer");
  EXPECT_NE(camera_transformer, nullptr);
  EXPECT_EQ(camera_transformer->Name(), "MyObstacleTransformer");
}

class MyObstaclePostprocessor : public BaseObstaclePostprocessor {
 public:
  MyObstaclePostprocessor() {}

  virtual ~MyObstaclePostprocessor() {}

  bool Init(const ObstaclePostprocessorInitOptions& options) override {
    return true;
  }

  bool Process(const ObstaclePostprocessorOptions& options,
               CameraFrame* frame) override {
    return true;
  }

  std::string Name() const override { return "MyObstaclePostprocessor"; }
};  // class MyObstaclePostprocessor

REGISTER_OBSTACLE_POSTPROCESSOR(MyObstaclePostprocessor);

TEST(ObstacleInterfaceTest, test_camera_postprocessor) {
  BaseObstaclePostprocessor* camera_postprocessor =
      BaseObstaclePostprocessorRegisterer::GetInstanceByName(
          "MyObstaclePostprocessor");
  EXPECT_NE(camera_postprocessor, nullptr);
  EXPECT_EQ(camera_postprocessor->Name(), "MyObstaclePostprocessor");
}

class MyTrafficLightDetector : public BaseTrafficLightDetector {
 public:
  MyTrafficLightDetector() {}

  virtual ~MyTrafficLightDetector() {}

  bool Init(const TrafficLightDetectorInitOptions& options) override {
    return true;
  }

  bool Detect(const TrafficLightDetectorOptions& options,
              CameraFrame* frame) override {
    return true;
  }

  std::string Name() const override { return "MyTrafficLightDetector"; }
};  // class MyTrafficLightDetector

REGISTER_TRAFFIC_LIGHT_DETECTOR(MyTrafficLightDetector);

TEST(TrafficLightInterfaceTest, test_traffic_light_detector) {
  BaseTrafficLightDetector* traffic_light_detector =
      BaseTrafficLightDetectorRegisterer::GetInstanceByName(
          "MyTrafficLightDetector");
  EXPECT_NE(traffic_light_detector, nullptr);
  EXPECT_EQ(traffic_light_detector->Name(), "MyTrafficLightDetector");
}

class MyTrafficLightTracker : public BaseTrafficLightTracker {
 public:
  MyTrafficLightTracker() {}

  virtual ~MyTrafficLightTracker() {}

  bool Init(const TrafficLightTrackerInitOptions& options) override {
    return true;
  }

  bool Track(const TrafficLightTrackerOptions& options,
             CameraFrame* frame) override {
    return true;
  }

  std::string Name() const override { return "MyTrafficLightTracker"; }
};  // class MyTrafficLightTracker

REGISTER_TRAFFIC_LIGHT_TRACKER(MyTrafficLightTracker);

TEST(TrafficLightInterfaceTest, test_traffic_light_tracker) {
  BaseTrafficLightTracker* traffic_light_tracker =
      BaseTrafficLightTrackerRegisterer::GetInstanceByName(
          "MyTrafficLightTracker");
  EXPECT_NE(traffic_light_tracker, nullptr);
  EXPECT_EQ(traffic_light_tracker->Name(), "MyTrafficLightTracker");
}

class MyLandmarkDetector : public BaseLandmarkDetector {
 public:
  MyLandmarkDetector() {}

  virtual ~MyLandmarkDetector() {}

  bool Init(const LandmarkDetectorInitOptions& options) override {
    return true;
  }

  bool Detect(const LandmarkDetectorOptions& options,
              CameraFrame* frame) override {
    return true;
  }

  std::string Name() const override { return "MyLandmarkDetector"; }
};  // class MyLandmarkDetector

REGISTER_LANDMARK_DETECTOR(MyLandmarkDetector);

TEST(LandmarkDetectorInterfaceTest, test_landmark_detector) {
  BaseLandmarkDetector* landmark_detector =
      BaseLandmarkDetectorRegisterer::GetInstanceByName("MyLandmarkDetector");
  EXPECT_NE(landmark_detector, nullptr);
  EXPECT_EQ(landmark_detector->Name(), "MyLandmarkDetector");
}

class MyLaneDetector : public BaseLaneDetector {
 public:
  MyLaneDetector() {}

  virtual ~MyLaneDetector() {}

  bool Init(const LaneDetectorInitOptions& options) override { return true; }

  bool Detect(const LaneDetectorOptions& options, CameraFrame* frame) override {
    return true;
  }

  std::string Name() const override { return "MyLaneDetector"; }
};  // class MyLaneDetector

REGISTER_LANE_DETECTOR(MyLaneDetector);

TEST(LaneInterfaceTest, test_lane_detector) {
  BaseLaneDetector* lane_detector =
      BaseLaneDetectorRegisterer::GetInstanceByName("MyLaneDetector");
  EXPECT_NE(lane_detector, nullptr);
  EXPECT_EQ(lane_detector->Name(), "MyLaneDetector");
}

class MyLaneTracker : public BaseLaneTracker {
 public:
  MyLaneTracker() {}

  virtual ~MyLaneTracker() {}

  bool Init(const LaneTrackerInitOptions& options) override { return true; }

  bool Track(const LaneTrackerOptions& options, CameraFrame* frame) override {
    return true;
  }

  std::string Name() const override { return "MyLaneTracker"; }
};  // class MyLaneTracker

REGISTER_LANE_TRACKER(MyLaneTracker);

TEST(LaneInterfaceTest, test_lane_tracker) {
  BaseLaneTracker* lane_tracker =
      BaseLaneTrackerRegisterer::GetInstanceByName("MyLaneTracker");
  EXPECT_NE(lane_tracker, nullptr);
  EXPECT_EQ(lane_tracker->Name(), "MyLaneTracker");
}

class MyLanePostprocessor : public BaseLanePostprocessor {
 public:
  MyLanePostprocessor() {}

  virtual ~MyLanePostprocessor() {}

  bool Init(const LanePostprocessorInitOptions& options) override {
    return true;
  }

  bool Process2D(const LanePostprocessorOptions& options,
                 CameraFrame* frame) override {
    return true;
  }

  bool Process3D(const LanePostprocessorOptions& options,
                 CameraFrame* frame) override {
    return true;
  }
  std::string Name() const override { return "MyLanePostprocessor"; }
};  // class MyLanePostprocessor

REGISTER_LANE_POSTPROCESSOR(MyLanePostprocessor);

TEST(LaneInterfaceTest, test_lane_postprocessor) {
  BaseLanePostprocessor* lane_postprocessor =
      BaseLanePostprocessorRegisterer::GetInstanceByName("MyLanePostprocessor");
  EXPECT_NE(lane_postprocessor, nullptr);
  EXPECT_EQ(lane_postprocessor->Name(), "MyLanePostprocessor");
}

class MyFeatureExtractor : public BaseFeatureExtractor {
 public:
  MyFeatureExtractor() {}

  virtual ~MyFeatureExtractor() {}

  bool Init(const FeatureExtractorInitOptions& init_options) override {
    return true;
  }

  bool Extract(const FeatureExtractorOptions& options,
               CameraFrame* frame) override {
    return true;
  }

  std::string Name() const override { return "MyFeatureExtractor"; }
};  // class MyFeatureExtractor

REGISTER_FEATURE_EXTRACTOR(MyFeatureExtractor);

TEST(FeatureExtractorInterfaceTest, test_feature_extractor) {
  BaseFeatureExtractor* feature_extractor =
      BaseFeatureExtractorRegisterer::GetInstanceByName("MyFeatureExtractor");
  EXPECT_NE(feature_extractor, nullptr);
  EXPECT_EQ(feature_extractor->Name(), "MyFeatureExtractor");

  const int image_width = 200;
  const int image_height = 200;
  feature_extractor->set_roi(0, 0, image_width, image_height);

  std::vector<std::shared_ptr<base::Object>> objects;
  {
    std::shared_ptr<base::Object> obj(new base::Object);
    obj->camera_supplement.box.xmin = 50;
    obj->camera_supplement.box.ymin = 50;
    obj->camera_supplement.box.xmax = 100;
    obj->camera_supplement.box.ymax = 100;

    objects.push_back(obj);
  }
  feature_extractor->encode_bbox(&objects);
  EXPECT_NEAR(objects[0]->camera_supplement.box.xmin, 0.25f, 1e-6);
  EXPECT_NEAR(objects[0]->camera_supplement.box.ymin, 0.25f, 1e-6);
  EXPECT_NEAR(objects[0]->camera_supplement.box.xmax, 0.5f, 1e-6);
  EXPECT_NEAR(objects[0]->camera_supplement.box.ymax, 0.5f, 1e-6);

  feature_extractor->decode_bbox(&objects);
  EXPECT_NEAR(objects[0]->camera_supplement.box.xmin, 50.f, 1e-6);
  EXPECT_NEAR(objects[0]->camera_supplement.box.ymin, 50.f, 1e-6);
  EXPECT_NEAR(objects[0]->camera_supplement.box.xmax, 100.f, 1e-6);
  EXPECT_NEAR(objects[0]->camera_supplement.box.ymax, 100.f, 1e-6);
}

class MySceneParser : public BaseSceneParser {
 public:
  MySceneParser() {}

  virtual ~MySceneParser() {}

  bool Init(const SceneParserInitOptions& options) override { return true; }

  bool Parse(const SceneParserOptions& options, CameraFrame* frame) override {
    return true;
  }

  std::string Name() const override { return "MySceneParser"; }
};  // class MySceneParser

REGISTER_SCENE_PARSER(MySceneParser);

TEST(SceneInterfaceTest, test_scene_parser) {
  BaseSceneParser* scene_parser =
      BaseSceneParserRegisterer::GetInstanceByName("MySceneParser");
  EXPECT_NE(scene_parser, nullptr);
  EXPECT_EQ(scene_parser->Name(), "MySceneParser");
}

class MyCalibrator : public BaseCalibrator {
 public:
  MyCalibrator() {}

  virtual ~MyCalibrator() {}

  bool Init(const CalibratorInitOptions& options) override { return true; }

  bool Calibrate(const CalibratorOptions& options,
                 float* pitch_angle) override {
    return true;
  }

  std::string Name() const override { return "MyCalibrator"; }
};  // class MyCalibrator

REGISTER_CALIBRATOR(MyCalibrator);

TEST(CalibratorInterfaceTest, test_calibrator) {
  BaseCalibrator* calibrator =
      BaseCalibratorRegisterer::GetInstanceByName("MyCalibrator");
  EXPECT_NE(calibrator, nullptr);
  EXPECT_EQ(calibrator->Name(), "MyCalibrator");
}

class MyCalibrationService : public BaseCalibrationService {
 public:
  MyCalibrationService() {}

  virtual ~MyCalibrationService() {}

  bool Init(const CalibrationServiceInitOptions& options) override {
    return true;
  }

  bool BuildIndex() override { return true; }

  std::string Name() const override { return "MyCalibrationService"; }
};  // class MyCalibrationService

REGISTER_CALIBRATION_SERVICE(MyCalibrationService);

TEST(CalibrationServiceInterfaceTest, test_calibrator) {
  BaseCalibrationService* calibrator =
      BaseCalibrationServiceRegisterer::GetInstanceByName(
          "MyCalibrationService");
  EXPECT_NE(calibrator, nullptr);
  EXPECT_EQ(calibrator->Name(), "MyCalibrationService");
}

class MyCameraPerception : public BaseCameraPerception {
 public:
  MyCameraPerception() {}
  virtual ~MyCameraPerception() {}

  bool Init(const CameraPerceptionInitOptions& init_options) override {
    return true;
  }
  bool Perception(const CameraPerceptionOptions& options,
                  CameraFrame* frame) override {
    return true;
  }

  std::string Name() const override { return "MyCameraPerception"; }
};  // class MyCameraPerception

PERCEPTION_REGISTER_CAMERA_PERCEPTION(MyCameraPerception);

TEST(CameraPerceptionInterfaceTest, test_camera_perception) {
  BaseCameraPerception* camera_perception =
      BaseCameraPerceptionRegisterer::GetInstanceByName("MyCameraPerception");
  EXPECT_NE(camera_perception, nullptr);
  EXPECT_EQ(camera_perception->Name(), "MyCameraPerception");
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
