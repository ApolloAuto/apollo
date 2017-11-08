// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
// @author erlangz(zhengwenchao@baidu.com)
// @date 2016/09/28 13:51:01
// @file projection_test.cpp
// @brief 
// 
#include <gtest/gtest.h>
#include "module/perception/traffic_light/projection/base_lights/projection.h"
#include "module/perception/traffic_light/projection/projection.h"
#include "module/perception/traffic_light/projection/multi_camera_projection.h"

namespace adu {
namespace perception {
namespace traffic_light {

class MultiCameraBoundaryBasedProjectionTest : public ::testing::Test {
 public:
  MultiCameraBoundaryBasedProjectionTest() :
      _projection(nullptr) {
  }
  ~MultiCameraBoundaryBasedProjectionTest() {
    delete _projection;
    _projection = nullptr;
  }
  virtual void SetUp() override {
    _projection = new MultiCamerasProjection;
    FLAGS_traffic_light_projection = "MultiCamerasProjection";
    ASSERT_TRUE(_projection);
    ASSERT_TRUE(_projection->init());
  }
  virtual void TearDown() override {
    delete _projection;
    _projection = nullptr;
  }

  void prepare_test_data(CarPose *pose) {
    prepare_test_data_long_focus(pose, nullptr);
  }

  void prepare_test_data_long_focus(CarPose *pose, Light *light) {

    pose->_pose << 0.928898, -0.370222, -0.00911767, 433843,
        0.37033, 0.928479, 0.0279876, 4.43584e+06,
        -0.00189607, -0.0293742, 0.999567, 34.2054,
        0, 0, 0, 1;

    if (!light) {
      return;
    }
    adu::common::hdmap::Signal &tl_info = light->info;

    // point { x: 433815.51701951429 y: 4435923.9110349407 z: 38.947281437191229 }
    // point { x: 433816.01932359143 y: 4435924.1800259342 z: 38.947281437191229 }
    // point { x: 433816.01932359143 y: 4435924.1800259342 z: 40.693268405470114 }
    // point { x: 433815.51701951429 y: 4435923.9110349407 z: 40.693268405470114 }
    // projection get ROI:[13 x 42 from (1057, 392)]
    double x1 = 433815.51701951429;
    double x2 = 433816.01932359143;
    double y1 = 4435923.9110349407;
    double y2 = 4435924.1800259342;
    double z1 = 38.947281437191229;
    double z2 = 40.693268405470114;

    /*
      (x1, y1, z1)
      (x2, y2, z1)
      (x2, y2, z2)
      (x1, y1, z2)
     */
    auto boundary = tl_info.mutable_boundary();
    auto pt = boundary->add_point();
    pt->set_x(x1);
    pt->set_y(y1);
    pt->set_z(z1);

    pt = boundary->add_point();
    pt->set_x(x2);
    pt->set_y(y2);
    pt->set_z(z1);

    pt = boundary->add_point();
    pt->set_x(x2);
    pt->set_y(y2);
    pt->set_z(z2);

    pt = boundary->add_point();
    pt->set_x(x1);
    pt->set_y(y1);
    pt->set_z(z2);
  }

  void prepare_test_data_short_focus(CarPose *pose, Light *light) {
    pose->_pose << -0.869575, 0.493604, -0.0139364, 429986,
        -0.493661, -0.869654, 0.000698665, 4.43728e+06,
        -0.011775, 0.00748738, 0.999903, 34.8932,
        0, 0, 0, 1;

    if (!light) {
      return;
    }
    adu::common::hdmap::Signal &tl_info = light->info;

    // point { x: 429996.05285042938 y: 4437244.7410057671 z: 40.240191 }
    // point { x: 429996.501825 y: 4437245.008157 z: 40.240191 }
    // point { x: 429996.501825 y: 4437245.008157 z: 41.869338 }
    // point { x: 429996.05285042938 y: 4437244.7410057671 z: 41.869338 }
    // projection get ROI:[39 x 97 from (1414, 507)]
    double x1 = 429996.05285042938;
    double x2 = 429996.501825;
    double y1 = 4437244.7410057671;
    double y2 = 4437245.008157;
    double z1 = 40.240191;
    double z2 = 41.869338;

    /*
      (x1, y1, z1)
      (x2, y2, z1)
      (x2, y2, z2)
      (x1, y1, z2)
     */
    auto boundary = tl_info.mutable_boundary();
    auto pt = boundary->add_point();
    pt->set_x(x1);
    pt->set_y(y1);
    pt->set_z(z1);

    pt = boundary->add_point();
    pt->set_x(x2);
    pt->set_y(y2);
    pt->set_z(z1);

    pt = boundary->add_point();
    pt->set_x(x2);
    pt->set_y(y2);
    pt->set_z(z2);

    pt = boundary->add_point();
    pt->set_x(x1);
    pt->set_y(y1);
    pt->set_z(z2);
  }

 private:
  MultiCamerasProjection *_projection;
};

TEST_F(MultiCameraBoundaryBasedProjectionTest, test_init) {
  FLAGS_traffic_light_projection = "MultiCamerasProjection";
  ASSERT_TRUE(_projection->init());

  FLAGS_traffic_light_projection = "DummyProjection";
  ASSERT_FALSE(_projection->init());

  FLAGS_traffic_light_projection = "FakeCameraProjection";
  ASSERT_FALSE(_projection->init());

  FLAGS_traffic_light_projection = "BadCameraParamFileProjection";
  ASSERT_FALSE(_projection->init());

  FLAGS_traffic_light_projection = "BadSingleProjection";
  ASSERT_FALSE(_projection->init());

  FLAGS_traffic_light_projection = "NoLongFocusProjection";
  ASSERT_FALSE(_projection->init());

  FLAGS_traffic_light_projection = "LongShortProjection";
  ASSERT_TRUE(_projection->init());
}

TEST_F(MultiCameraBoundaryBasedProjectionTest, test_project_long_focus) {
  FLAGS_traffic_light_projection = "MultiCamerasProjection";
  if (_projection) {
    delete _projection;
    _projection = NULL;
  }
  _projection = new MultiCamerasProjection;
  ASSERT_TRUE(_projection);
  ASSERT_TRUE(_projection->init());

  CarPose pose;
  Light light;
  prepare_test_data_long_focus(&pose, &light);
  ASSERT_TRUE(_projection->project(pose, ProjectOption(LONG_FOCUS), &light));
  EXPECT_EQ(1038, light.region.projection_roi.x);
  EXPECT_EQ(577, light.region.projection_roi.y);
  EXPECT_EQ(50, light.region.projection_roi.width);
  EXPECT_EQ(157, light.region.projection_roi.height);
}

TEST_F(MultiCameraBoundaryBasedProjectionTest, test_project_short_focus) {
  FLAGS_traffic_light_projection = "MultiCamerasProjection";
  if (_projection) {
    delete _projection;
    _projection = NULL;
  }
  _projection = new MultiCamerasProjection;
  ASSERT_TRUE(_projection);
  ASSERT_TRUE(_projection->init());

  CarPose pose;
  Light light;
  prepare_test_data_short_focus(&pose, &light);
  ASSERT_TRUE(_projection->project(pose, ProjectOption(SHORT_FOCUS), &light));
  EXPECT_EQ(1411, light.region.projection_roi.x);
  EXPECT_EQ(248, light.region.projection_roi.y);
  EXPECT_EQ(33, light.region.projection_roi.width);
  EXPECT_EQ(90, light.region.projection_roi.height);
}

TEST_F(MultiCameraBoundaryBasedProjectionTest, test_project_narrow_focus) {
  FLAGS_traffic_light_projection = "MultiCamerasProjection";
  if (_projection) {
    delete _projection;
    _projection = NULL;
  }
  _projection = new MultiCamerasProjection;
  ASSERT_TRUE(_projection);
  ASSERT_TRUE(_projection->init());

  CarPose pose;
  Light light;
  // need real data; now use short focus test data
  prepare_test_data_short_focus(&pose, &light);
  ASSERT_TRUE(_projection->project(pose, ProjectOption(CameraId::NARROW_FOCUS), &light));
  EXPECT_EQ(1411, light.region.projection_roi.x);
  EXPECT_EQ(248, light.region.projection_roi.y);
  EXPECT_EQ(33, light.region.projection_roi.width);
  EXPECT_EQ(90, light.region.projection_roi.height);
}

TEST_F(MultiCameraBoundaryBasedProjectionTest, test_project_wide_focus) {
  FLAGS_traffic_light_projection = "MultiCamerasProjection";
  if (_projection) {
    delete _projection;
    _projection = NULL;
  }
  _projection = new MultiCamerasProjection;
  ASSERT_TRUE(_projection);
  ASSERT_TRUE(_projection->init());

  CarPose pose;
  Light light;
  // need real data; now use short focus test data
  prepare_test_data_short_focus(&pose, &light);
  ASSERT_TRUE(_projection->project(pose, ProjectOption(CameraId::WIDE_FOCUS), &light));
  EXPECT_EQ(1411, light.region.projection_roi.x);
  EXPECT_EQ(248, light.region.projection_roi.y);
  EXPECT_EQ(33, light.region.projection_roi.width);
  EXPECT_EQ(90, light.region.projection_roi.height);
}

TEST_F(MultiCameraBoundaryBasedProjectionTest, test_project_no_wide) {
  FLAGS_traffic_light_projection = "MultiCamerasProjection_NoWideCamera";
  if (_projection) {
    delete _projection;
    _projection = NULL;
  }
  _projection = new MultiCamerasProjection;
  ASSERT_TRUE(_projection);
  ASSERT_TRUE(_projection->init());

  CarPose pose;
  Light light;
  // need real data; now use short focus test data
  prepare_test_data_short_focus(&pose, &light);
  ASSERT_FALSE(_projection->project(pose, ProjectOption(CameraId::WIDE_FOCUS), &light));

  ASSERT_TRUE(_projection->project(pose, ProjectOption(SHORT_FOCUS), &light));
  EXPECT_EQ(1411, light.region.projection_roi.x);
  EXPECT_EQ(248, light.region.projection_roi.y);
  EXPECT_EQ(33, light.region.projection_roi.width);
  EXPECT_EQ(90, light.region.projection_roi.height);
}

TEST_F(MultiCameraBoundaryBasedProjectionTest, test_project_no_narrow) {
  FLAGS_traffic_light_projection = "MultiCamerasProjection_NoNarrowCamera";
  if (_projection) {
    delete _projection;
    _projection = NULL;
  }
  _projection = new MultiCamerasProjection;
  ASSERT_TRUE(_projection);
  ASSERT_TRUE(_projection->init());

  CarPose pose;
  Light light;
  // need real data; now use short focus test data
  prepare_test_data_short_focus(&pose, &light);
  ASSERT_FALSE(_projection->project(pose, ProjectOption(CameraId::NARROW_FOCUS), &light));

  ASSERT_TRUE(_projection->project(pose, ProjectOption(SHORT_FOCUS), &light));
  EXPECT_EQ(1411, light.region.projection_roi.x);
  EXPECT_EQ(248, light.region.projection_roi.y);
  EXPECT_EQ(33, light.region.projection_roi.width);
  EXPECT_EQ(90, light.region.projection_roi.height);
}

TEST_F(MultiCameraBoundaryBasedProjectionTest, test_project_unknown_camera) {
  FLAGS_traffic_light_projection = "MultiCamerasProjection";
  if (_projection) {
    delete _projection;
    _projection = NULL;
  }
  _projection = new MultiCamerasProjection;
  ASSERT_TRUE(_projection);
  ASSERT_TRUE(_projection->init());

  CarPose pose;
  Light light;
  prepare_test_data_short_focus(&pose, &light);
  ASSERT_FALSE(_projection->project(pose, ProjectOption(UNKNOWN), &light));
}

TEST_F(MultiCameraBoundaryBasedProjectionTest, test_project_fails) {
  FLAGS_traffic_light_projection = "MultiCamerasProjection";
  if (_projection) {
    delete _projection;
    _projection = NULL;
  }
  _projection = new MultiCamerasProjection;
  ASSERT_TRUE(_projection);
  ASSERT_TRUE(_projection->init());

  CarPose pose;
  Light light;
  prepare_test_data_long_focus(&pose, &light);

  pose._pose = Eigen::Matrix4d::Identity();
  ASSERT_FALSE(_projection->project(pose, ProjectOption(LONG_FOCUS), &light));
}

class SingleBoundaryBasedProjectionTest : public ::testing::Test {
 public:
  SingleBoundaryBasedProjectionTest() :
      _camera_coeffients(nullptr) {
  }
  ~SingleBoundaryBasedProjectionTest() {
    delete _camera_coeffients;
  }
  virtual void SetUp() {
    _camera_coeffients = new CameraCoeffient;
    ASSERT_TRUE(_camera_coeffients);

    std::string lidar2gps_file = "./data/multi_projection/velodyne64_novatel_extrinsics.yaml";
    std::string lidar2camera_file = "./data/multi_projection/onsemi_obstacle_extrinsics.yaml";
    std::string camera_intrinsic_file =
        "./data/multi_projection/onsemi_obstacle_intrinsics.yaml";
    ASSERT_TRUE(_camera_coeffients->init("camera_6mm_focus",
                                         lidar2gps_file,
                                         lidar2camera_file,
                                         camera_intrinsic_file));

    _camera_coeffients->gps2camera =
        _camera_coeffients->lidar2gps * _camera_coeffients->lidar2camera;
    _camera_coeffients->gps2camera = _camera_coeffients->gps2camera.inverse();
  }
  virtual void TearDown() {
    delete _camera_coeffients;
    _camera_coeffients = nullptr;
  }
  void prepare_test_data(CarPose *pose, adu::common::hdmap::Signal *tl_info) {
    pose->_pose << -0.869575, 0.493604, -0.0139364, 429986,
        -0.493661, -0.869654, 0.000698665, 4.43728e+06,
        -0.011775, 0.00748738, 0.999903, 34.8932,
        0, 0, 0, 1;

    if (!tl_info) {
      return;
    }

    // point { x: 429996.05285042938 y: 4437244.7410057671 z: 40.240191 }
    // point { x: 429996.501825 y: 4437245.008157 z: 40.240191 }
    // point { x: 429996.501825 y: 4437245.008157 z: 41.869338 }
    // point { x: 429996.05285042938 y: 4437244.7410057671 z: 41.869338 }
    double x1 = 429996.05285042938;
    double x2 = 429996.501825;
    double y1 = 4437244.7410057671;
    double y2 = 4437245.008157;
    double z1 = 40.240191;
    double z2 = 41.869338;

    /*
      (x1, y1, z1)
      (x2, y2, z1)
      (x2, y2, z2)
      (x1, y1, z2)
     */
    auto boundary = tl_info->mutable_boundary();
    auto pt = boundary->add_point();
    pt->set_x(x1);
    pt->set_y(y1);
    pt->set_z(z1);

    pt = boundary->add_point();
    pt->set_x(x2);
    pt->set_y(y2);
    pt->set_z(z1);

    pt = boundary->add_point();
    pt->set_x(x2);
    pt->set_y(y2);
    pt->set_z(z2);

    pt = boundary->add_point();
    pt->set_x(x1);
    pt->set_y(y1);
    pt->set_z(z2);
  }

  void prepare_test_data_wrong_boundary_size(
      CarPose *pose, adu::common::hdmap::Signal *tl_info) {
    pose->_pose << -0.869575, 0.493604, -0.0139364, 429986,
        -0.493661, -0.869654, 0.000698665, 4.43728e+06,
        -0.011775, 0.00748738, 0.999903, 34.8932,
        0, 0, 0, 1;

    if (!tl_info) {
      return;
    }

    // point { x: 429996.05285042938 y: 4437244.7410057671 z: 40.240191 }
    // point { x: 429996.501825 y: 4437245.008157 z: 40.240191 }
    // point { x: 429996.501825 y: 4437245.008157 z: 41.869338 }
    // point { x: 429996.05285042938 y: 4437244.7410057671 z: 41.869338 }
    double x1 = 429996.05285042938;
    double x2 = 429996.501825;
    double y1 = 4437244.7410057671;
    double y2 = 4437245.008157;
    double z1 = 40.240191;
    double z2 = 41.869338;

    /*
      (x1, y1, z1)
      (x2, y2, z1)
      (x2, y2, z2)
      (x1, y1, z2)
     */
    auto boundary = tl_info->mutable_boundary();
    auto pt = boundary->add_point();
    pt->set_x(x1);
    pt->set_y(y1);
    pt->set_z(z1);

    pt = boundary->add_point();
    pt->set_x(x2);
    pt->set_y(y2);
    pt->set_z(z1);

    pt = boundary->add_point();
    pt->set_x(x2);
    pt->set_y(y2);
    pt->set_z(z2);
  }

 private:
  SingleBoundaryBasedProjection _projection;
  CameraCoeffient *_camera_coeffients;
};

TEST_F(SingleBoundaryBasedProjectionTest, init_fail) {
  CameraCoeffient camera_coeffients;

  ASSERT_FALSE(camera_coeffients.init("Test Focus",
                                      "./data/projection/not-exists.yaml",
                                      "./data/projection/not-exists.yaml",
                                      "./data/projection/not-exists.yaml"));

  ASSERT_FALSE(camera_coeffients.init("Test Focus",
                                      "./data/projection/not-exists.yaml",
                                      "./data/projection/not-exists.yaml",
                                      "./data/projection/not-exists.yaml"));
  ASSERT_FALSE(camera_coeffients.init("Test Focus",
                                      "./data/projection/empty.yaml",
                                      "./data/projection/not-exists.yaml",
                                      "./data/projection/not-exists.yaml"));
  ASSERT_FALSE(camera_coeffients.init("Test Focus",
                                      "./data/projection/velodyne64_novatel.yaml",
                                      "./data/projection/empty.yaml",
                                      "./data/projection/empty.yaml"));
  ASSERT_FALSE(camera_coeffients.init("Test Focus",
                                      "./data/projection/velodyne64_novatel.yaml",
                                      "./data/projection/camera.yaml",
                                      "./data/projection/empty.yaml"));
}

TEST_F(SingleBoundaryBasedProjectionTest, test_project) {
  CarPose pose;
  adu::common::hdmap::Signal tl_info;

  // wrong boundary size
  prepare_test_data_wrong_boundary_size(&pose, &tl_info);
  Light light;
  ASSERT_FALSE(_projection.project(*_camera_coeffients, pose.get_pose(), tl_info, &light));

  prepare_test_data(&pose, &tl_info);
  ASSERT_TRUE(_projection.project(*_camera_coeffients, pose.get_pose(), tl_info, &light));

  auto pose_ahead_light = pose.get_pose();
  pose_ahead_light(0, 3) = pose_ahead_light(0, 3) + 1000;
  CarPose car_pose_ahead_light;
  car_pose_ahead_light.init(pose_ahead_light);
  ASSERT_FALSE(_projection.project(*_camera_coeffients,
                                   car_pose_ahead_light.get_pose(), tl_info, &light));

  auto wrong_pose = pose.get_pose();
  wrong_pose(1, 3) = wrong_pose(1, 3) - 10000;
  CarPose wrong_car_pose;
  wrong_car_pose.init(wrong_pose);
  ASSERT_FALSE(_projection.project(*_camera_coeffients,
                                   wrong_car_pose.get_pose(), tl_info, &light));

  _camera_coeffients->image_width = 1;
  _camera_coeffients->image_height = 1080;
  ASSERT_FALSE(_projection.project(*_camera_coeffients, pose.get_pose(), tl_info, &light));

  _camera_coeffients->image_width = 1920;
  _camera_coeffients->image_height = 1;
  ASSERT_FALSE(_projection.project(*_camera_coeffients, pose.get_pose(), tl_info, &light));

  _camera_coeffients->image_width = 1920;
  _camera_coeffients->image_height = 1080;
  _camera_coeffients->gps2camera = Eigen::Matrix4d::Identity();
  ASSERT_FALSE(_projection.project(*_camera_coeffients, pose.get_pose(), tl_info, &light));
}

class TwoCamerasBoundaryBasedProjectionTest : public ::testing::Test {
 public:
  TwoCamerasBoundaryBasedProjectionTest() :
      _projection(nullptr) {
  }
  ~TwoCamerasBoundaryBasedProjectionTest() {
    delete _projection;
    _projection = nullptr;
  }
  virtual void SetUp() override {
    _projection = new TwoCamerasProjection;
    FLAGS_traffic_light_projection = "SingleBoundaryBasedProjection";
    ASSERT_TRUE(_projection);
    ASSERT_TRUE(_projection->init());
  }
  virtual void TearDown() override {
    delete _projection;
    _projection = nullptr;
  }

  void prepare_test_data(CarPose *pose) {
    prepare_test_data_long_focus(pose, nullptr);
  }

  void prepare_test_data_long_focus(CarPose *pose, Light *light) {

    pose->_pose << 0.928898, -0.370222, -0.00911767, 433843,
        0.37033, 0.928479, 0.0279876, 4.43584e+06,
        -0.00189607, -0.0293742, 0.999567, 34.2054,
        0, 0, 0, 1;

    if (!light) {
      return;
    }
    adu::common::hdmap::Signal &tl_info = light->info;

    // point { x: 433815.51701951429 y: 4435923.9110349407 z: 38.947281437191229 }
    // point { x: 433816.01932359143 y: 4435924.1800259342 z: 38.947281437191229 }
    // point { x: 433816.01932359143 y: 4435924.1800259342 z: 40.693268405470114 }
    // point { x: 433815.51701951429 y: 4435923.9110349407 z: 40.693268405470114 }
    // projection get ROI:[13 x 42 from (1057, 392)]
    double x1 = 433815.51701951429;
    double x2 = 433816.01932359143;
    double y1 = 4435923.9110349407;
    double y2 = 4435924.1800259342;
    double z1 = 38.947281437191229;
    double z2 = 40.693268405470114;

    /*
      (x1, y1, z1)
      (x2, y2, z1)
      (x2, y2, z2)
      (x1, y1, z2)
     */
    auto boundary = tl_info.mutable_boundary();
    auto pt = boundary->add_point();
    pt->set_x(x1);
    pt->set_y(y1);
    pt->set_z(z1);

    pt = boundary->add_point();
    pt->set_x(x2);
    pt->set_y(y2);
    pt->set_z(z1);

    pt = boundary->add_point();
    pt->set_x(x2);
    pt->set_y(y2);
    pt->set_z(z2);

    pt = boundary->add_point();
    pt->set_x(x1);
    pt->set_y(y1);
    pt->set_z(z2);
  }

  void prepare_test_data_short_focus(CarPose *pose, Light *light) {
    pose->_pose << -0.869575, 0.493604, -0.0139364, 429986,
        -0.493661, -0.869654, 0.000698665, 4.43728e+06,
        -0.011775, 0.00748738, 0.999903, 34.8932,
        0, 0, 0, 1;

    if (!light) {
      return;
    }
    adu::common::hdmap::Signal &tl_info = light->info;

    // point { x: 429996.05285042938 y: 4437244.7410057671 z: 40.240191 }
    // point { x: 429996.501825 y: 4437245.008157 z: 40.240191 }
    // point { x: 429996.501825 y: 4437245.008157 z: 41.869338 }
    // point { x: 429996.05285042938 y: 4437244.7410057671 z: 41.869338 }
    // projection get ROI:[39 x 97 from (1414, 507)]
    double x1 = 429996.05285042938;
    double x2 = 429996.501825;
    double y1 = 4437244.7410057671;
    double y2 = 4437245.008157;
    double z1 = 40.240191;
    double z2 = 41.869338;

    /*
      (x1, y1, z1)
      (x2, y2, z1)
      (x2, y2, z2)
      (x1, y1, z2)
     */
    auto boundary = tl_info.mutable_boundary();
    auto pt = boundary->add_point();
    pt->set_x(x1);
    pt->set_y(y1);
    pt->set_z(z1);

    pt = boundary->add_point();
    pt->set_x(x2);
    pt->set_y(y2);
    pt->set_z(z1);

    pt = boundary->add_point();
    pt->set_x(x2);
    pt->set_y(y2);
    pt->set_z(z2);

    pt = boundary->add_point();
    pt->set_x(x1);
    pt->set_y(y1);
    pt->set_z(z2);
  }

 private:
  TwoCamerasProjection *_projection;
};

TEST_F(TwoCamerasBoundaryBasedProjectionTest, test_init) {
  FLAGS_traffic_light_projection = "SingleLightsBasedProjection";
  ASSERT_FALSE(_projection->init());

  FLAGS_traffic_light_projection = "SingleBoundaryBasedProjection";
  ASSERT_TRUE(_projection->init());

  FLAGS_traffic_light_projection = "FakeProjection";
  ASSERT_FALSE(_projection->init());

  FLAGS_traffic_light_projection = "TwoCameraBadShortAngleProjection";
  ASSERT_FALSE(_projection->init());

  FLAGS_traffic_light_projection = "TwoCameraBadLongAngleProjection";
  ASSERT_FALSE(_projection->init());

  FLAGS_traffic_light_projection = "TwoCameraBadShortCoefficientProjection";
  ASSERT_FALSE(_projection->init());

  FLAGS_traffic_light_projection = "TwoCameraBadLongCoefficientProjection";
  ASSERT_FALSE(_projection->init());
}
TEST_F(TwoCamerasBoundaryBasedProjectionTest, test_project_long_focus) {
  FLAGS_traffic_light_projection = "SingleBoundaryBasedProjection";
  if (_projection) {
    delete _projection;
    _projection = NULL;
  }
  _projection = new TwoCamerasProjection;
  ASSERT_TRUE(_projection);
  ASSERT_TRUE(_projection->init());

  CarPose pose;
  Light light;
  prepare_test_data_long_focus(&pose, &light);
  ASSERT_TRUE(_projection->project(pose, ProjectOption(LONG_FOCUS), &light));
  EXPECT_EQ(1038, light.region.projection_roi.x);
  EXPECT_EQ(577, light.region.projection_roi.y);
  EXPECT_EQ(50, light.region.projection_roi.width);
  EXPECT_EQ(157, light.region.projection_roi.height);
}

TEST_F(TwoCamerasBoundaryBasedProjectionTest, test_project_short_focus) {
  FLAGS_traffic_light_projection = "SingleBoundaryBasedProjection";
  if (_projection) {
    delete _projection;
    _projection = NULL;
  }
  _projection = new TwoCamerasProjection;
  ASSERT_TRUE(_projection);
  ASSERT_TRUE(_projection->init());

  CarPose pose;
  Light light;
  prepare_test_data_short_focus(&pose, &light);
  ASSERT_TRUE(_projection->project(pose, ProjectOption(SHORT_FOCUS), &light));
  EXPECT_EQ(1411, light.region.projection_roi.x);
  EXPECT_EQ(248, light.region.projection_roi.y);
  EXPECT_EQ(33, light.region.projection_roi.width);
  EXPECT_EQ(90, light.region.projection_roi.height);
}

} // namespace traffic_light
} // namespace perception
} // namespace adu
