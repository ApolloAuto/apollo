#include "gtest/gtest.h"
#include "modules/perception/traffic_light/projection/base_lights/projection.h"
#include "modules/perception/traffic_light/projection/projection.h"
#include "modules/perception/traffic_light/projection/multi_camera_projection.h"
#include "modules/perception/traffic_light/preprocessor/tl_preprocessor.h"

namespace adu {
namespace perception {
namespace config_manager {
DECLARE_string(config_manager_path);
}

namespace traffic_light {

REGISTER_PROJECTION(SingleBoundaryBasedProjection);

class TLPreprocessorTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    if (base::Singleton<config_manager::ConfigManager>::_instance) {
      delete base::Singleton<config_manager::ConfigManager>::_instance;
      base::Singleton<config_manager::ConfigManager>::_instance = NULL;
    }
    base::Singleton<config_manager::ConfigManager>::_instance =
        new config_manager::ConfigManager;
    _preprocessor = new TLPreprocessor;
  }

  ~TLPreprocessorTest() {
    delete _preprocessor;
  }

  void prepare_test_data_long_focus(CarPose *pose, adu::common::hdmap::Signal *signals) {

    pose->_pose << 0.928898, -0.370222, -0.00911767, 433843,
        0.37033, 0.928479, 0.0279876, 4.43584e+06,
        -0.00189607, -0.0293742, 0.999567, 34.2054,
        0, 0, 0, 1;

    if (!signals) {
      return;
    }

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
    auto boundary = signals->mutable_boundary();
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

  void prepare_test_data_short_focus(CarPose *pose, adu::common::hdmap::Signal *signals) {
    pose->_pose << -0.869575, 0.493604, -0.0139364, 429986,
        -0.493661, -0.869654, 0.000698665, 4.43728e+06,
        -0.011775, 0.00748738, 0.999903, 34.8932,
        0, 0, 0, 1;

    if (!signals) {
      return;
    }

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
    auto boundary = signals->mutable_boundary();
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

 protected:
  TLPreprocessor *_preprocessor;
};

TEST_F(TLPreprocessorTest, test_init_fail) {
  config_manager::fLS::FLAGS_config_manager_path = "non-exist";
  ASSERT_FALSE(_preprocessor->init());
}

TEST_F(TLPreprocessorTest, test_init_exception) {
  config_manager::fLS::FLAGS_config_manager_path = "conf/bad_config_manager.config";
  ASSERT_FALSE(_preprocessor->init());
}

TEST_F(TLPreprocessorTest, test_init) {
  config_manager::fLS::FLAGS_config_manager_path = "conf/config_manager.config";
  ASSERT_TRUE(_preprocessor->init());
}

TEST_F(TLPreprocessorTest, test_add_cached_lights_projections) {
  ASSERT_TRUE(_preprocessor->init());
  CarPose pose;
  std::vector<adu::common::hdmap::Signal> signals;
  MultiCamerasProjection projection;
  std::map<int, int> image_borders = {
      {static_cast<int>(CameraId::LONG_FOCUS), 100},
      {static_cast<int>(CameraId::SHORT_FOCUS), 100},
      {static_cast<int>(CameraId::NARROW_FOCUS), 100},
      {static_cast<int>(CameraId::WIDE_FOCUS), 100}
  };
  double timestamp = 0.0;
  bool lights_projections_all_outside_image = false;
  FLAGS_traffic_light_projection = "MultiCamerasProjection";
  ASSERT_TRUE(projection.init());
  for (size_t i = 0; i < 300; ++i) {
    ASSERT_TRUE(_preprocessor->add_cached_lights_projections(
        pose, signals, projection, image_borders, timestamp,
        &lights_projections_all_outside_image));
  }

  // add light
  adu::common::hdmap::Signal tl_signal;
  double x1 = 433815.51701951429;
  double x2 = 433816.01932359143;
  double y1 = 4435923.9110349407;
  double y2 = 4435924.1800259342;
  double z1 = 38.947281437191229;
  double z2 = 40.693268405470114;

  auto boundary = tl_signal.mutable_boundary();
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

  signals.push_back(tl_signal);
  for (size_t i = 0; i < 300; ++i) {
    ASSERT_TRUE(_preprocessor->add_cached_lights_projections(
        pose, signals, projection, image_borders, timestamp,
        &lights_projections_all_outside_image));
  }

  // valid pose
  pose._pose << 0.928898, -0.370222, -0.00911767, 433843,
      0.37033, 0.928479, 0.0279876, 4.43584e+06,
      -0.00189607, -0.0293742, 0.999567, 34.2054,
      0, 0, 0, 1;
  _preprocessor->set_camera_is_working_flag(CameraId::LONG_FOCUS, true);
  _preprocessor->set_camera_is_working_flag(CameraId::SHORT_FOCUS, true);
  for (size_t i = 0; i < 300; ++i) {
    ASSERT_TRUE(_preprocessor->add_cached_lights_projections(
        pose, signals, projection, image_borders, timestamp,
        &lights_projections_all_outside_image));
  }
}

TEST_F(TLPreprocessorTest, test_is_in_bord) {
  ASSERT_TRUE(_preprocessor->init());
  cv::Size image_size(400, 300);
  int border_size = 50;

  cv::Rect roi1(0, 0, 20, 20);
  ASSERT_TRUE(_preprocessor->is_in_bord(image_size, roi1, border_size));

  cv::Rect roi2(0, 100, 20, 20);
  ASSERT_TRUE(_preprocessor->is_in_bord(image_size, roi2, border_size));

  cv::Rect roi3(100, 0, 20, 20);
  ASSERT_TRUE(_preprocessor->is_in_bord(image_size, roi3, border_size));

  cv::Rect roi4(300, 200, 100, 100);
  ASSERT_TRUE(_preprocessor->is_in_bord(image_size, roi4, border_size));

  cv::Rect roi5(300, 200, 10, 100);
  ASSERT_TRUE(_preprocessor->is_in_bord(image_size, roi5, border_size));

  cv::Rect roi6(300, 200, 100, 10);
  ASSERT_TRUE(_preprocessor->is_in_bord(image_size, roi6, border_size));
}

TEST_F(TLPreprocessorTest, test_set_and_get_camera_is_working_flag) {
  ASSERT_TRUE(_preprocessor->init());
  int cam_id = -1;
  bool in_use = false;
  ASSERT_FALSE(_preprocessor->set_camera_is_working_flag(static_cast<CameraId>(cam_id), true));
  ASSERT_FALSE(_preprocessor->set_camera_is_working_flag(static_cast<CameraId>(cam_id), false));
  ASSERT_FALSE(_preprocessor->get_camera_is_working_flag(static_cast<CameraId>(cam_id), &in_use));
  ASSERT_FALSE(in_use);

  cam_id = 100;
  in_use = false;
  ASSERT_FALSE(_preprocessor->set_camera_is_working_flag(static_cast<CameraId>(cam_id), true));
  ASSERT_FALSE(_preprocessor->set_camera_is_working_flag(static_cast<CameraId>(cam_id), false));
  ASSERT_FALSE(_preprocessor->get_camera_is_working_flag(static_cast<CameraId>(cam_id), &in_use));
  ASSERT_FALSE(in_use);

  cam_id = 1;
  in_use = false;
  ASSERT_TRUE(_preprocessor->set_camera_is_working_flag(static_cast<CameraId>(cam_id), true));
  ASSERT_TRUE(_preprocessor->set_camera_is_working_flag(static_cast<CameraId>(cam_id), false));
  ASSERT_TRUE(_preprocessor->set_camera_is_working_flag(static_cast<CameraId>(cam_id), true));
  ASSERT_TRUE(_preprocessor->get_camera_is_working_flag(static_cast<CameraId>(cam_id), &in_use));
  ASSERT_TRUE(in_use);

  cam_id = 0;
  in_use = false;
  ASSERT_TRUE(_preprocessor->get_camera_is_working_flag(static_cast<CameraId>(cam_id), &in_use));
  ASSERT_FALSE(in_use);
}

TEST_F(TLPreprocessorTest, test_sync_image_with_cached_lights_projections) {
  ASSERT_TRUE(_preprocessor->init());
  std::shared_ptr<Image> image(new Image);
  int cam_id = -1;
  double sync_time = 0.0;
  std::shared_ptr<ImageLights> data(new ImageLights);
  double diff_image_pose_ts = 0.0;
  double diff_image_sys_ts = 0.0;
  bool sync_ok = false;

  ASSERT_FALSE(_preprocessor->sync_image_with_cached_lights_projections(
      image,
      static_cast<CameraId>(cam_id),
      sync_time,
      data,
      &diff_image_pose_ts,
      &diff_image_sys_ts,
      &sync_ok));

  cam_id = 1000;
  ASSERT_FALSE(_preprocessor->sync_image_with_cached_lights_projections(
      image,
      static_cast<CameraId>(cam_id),
      sync_time,
      data,
      &diff_image_pose_ts,
      &diff_image_sys_ts,
      &sync_ok));

  cam_id = 0;
  ASSERT_TRUE(_preprocessor->sync_image_with_cached_lights_projections(
      image,
      static_cast<CameraId>(cam_id),
      sync_time,
      data,
      &diff_image_pose_ts,
      &diff_image_sys_ts,
      &sync_ok));

  cam_id = 0;
  diff_image_pose_ts = 0.0;
  sync_ok = false;
  {
    std::shared_ptr<ImageLights> image_lights(new ImageLights);
    image_lights->timestamp = 0.2;
    image_lights->camera_id = CameraId::UNKNOWN;
    _preprocessor->_cached_lights_projections_array.push_back(image_lights);
    ASSERT_TRUE(_preprocessor->sync_image_with_cached_lights_projections(
        image,
        static_cast<CameraId>(cam_id),
        sync_time,
        data,
        &diff_image_pose_ts,
        &diff_image_sys_ts,
        &sync_ok));
    ASSERT_FALSE(sync_ok);
    _preprocessor->_cached_lights_projections_array.clear();
  }

  cam_id = 0;
  diff_image_pose_ts = 0.0;
  sync_ok = false;
  {
    std::shared_ptr<ImageLights> image_lights(new ImageLights);
    image_lights->timestamp = 0.01;
    image_lights->camera_id = CameraId::UNKNOWN;
    _preprocessor->_cached_lights_projections_array.push_back(image_lights);
    ASSERT_TRUE(_preprocessor->sync_image_with_cached_lights_projections(
        image,
        static_cast<CameraId>(cam_id),
        sync_time,
        data,
        &diff_image_pose_ts,
        &diff_image_sys_ts,
        &sync_ok));
    ASSERT_FALSE(sync_ok);
    _preprocessor->_cached_lights_projections_array.clear();
  }

  cam_id = 0;
  diff_image_pose_ts = 0.0;
  sync_ok = false;
  {
    std::shared_ptr<ImageLights> image_lights(new ImageLights);
    image_lights->timestamp = 0.01;
    image_lights->camera_id = CameraId::LONG_FOCUS;
    _preprocessor->_cached_lights_projections_array.push_back(image_lights);
    ASSERT_TRUE(_preprocessor->sync_image_with_cached_lights_projections(
        image,
        static_cast<CameraId>(cam_id),
        sync_time,
        data,
        &diff_image_pose_ts,
        &diff_image_sys_ts,
        &sync_ok));
    ASSERT_TRUE(sync_ok);
    _preprocessor->_cached_lights_projections_array.clear();
  }

  cam_id = 0;
  diff_image_pose_ts = 0.0;
  sync_ok = false;
  {
    std::shared_ptr<ImageLights> image_lights(new ImageLights);
    image_lights->timestamp = 0.01;
    image_lights->camera_id = CameraId::LONG_FOCUS;
    _preprocessor->_cached_lights_projections_array.push_back(image_lights);
    _preprocessor->_last_output_ts = 1000.0;
    ASSERT_FALSE(_preprocessor->sync_image_with_cached_lights_projections(
        image,
        static_cast<CameraId>(cam_id),
        sync_time,
        data,
        &diff_image_pose_ts,
        &diff_image_sys_ts,
        &sync_ok));
    ASSERT_FALSE(sync_ok);
    _preprocessor->_cached_lights_projections_array.clear();
    _preprocessor->_last_output_ts = 0.0;
  }

  cam_id = 0;
  diff_image_pose_ts = 0.0;
  sync_ok = false;
  {
    std::shared_ptr<ImageLights> image_lights(new ImageLights);
    image_lights->timestamp = 0.01;
    image_lights->camera_id = CameraId::LONG_FOCUS;
    _preprocessor->_cached_lights_projections_array.push_back(image_lights);
    std::shared_ptr<ImageLights> data_null;
    ASSERT_TRUE(_preprocessor->sync_image_with_cached_lights_projections(
        image,
        static_cast<CameraId>(cam_id),
        sync_time,
        data_null,
        &diff_image_pose_ts,
        &diff_image_sys_ts,
        &sync_ok));
    ASSERT_TRUE(sync_ok);
    _preprocessor->_cached_lights_projections_array.clear();
  }

  cam_id = 0;
  diff_image_pose_ts = 0.0;
  sync_ok = false;
  {
    std::shared_ptr<ImageLights> image_lights1(new ImageLights);
    image_lights1->timestamp = -0.5;
    image_lights1->camera_id = CameraId::LONG_FOCUS;
    _preprocessor->_cached_lights_projections_array.push_back(image_lights1);

    std::shared_ptr<ImageLights> image_lights2(new ImageLights);
    image_lights2->timestamp = 0.7;
    image_lights2->camera_id = CameraId::LONG_FOCUS;
    _preprocessor->_cached_lights_projections_array.push_back(image_lights2);

    ASSERT_TRUE(_preprocessor->sync_image_with_cached_lights_projections(
        image,
        static_cast<CameraId>(cam_id),
        sync_time,
        data,
        &diff_image_pose_ts,
        &diff_image_sys_ts,
        &sync_ok));
    ASSERT_FALSE(sync_ok);
    EXPECT_DOUBLE_EQ(0.0, diff_image_pose_ts);
    _preprocessor->_cached_lights_projections_array.clear();
    _preprocessor->_last_output_ts = 0.0;
  }

  cam_id = 0;
  diff_image_pose_ts = 0.0;
  sync_ok = false;
  {
    std::shared_ptr<ImageLights> image_lights1(new ImageLights);
    image_lights1->timestamp = 0.2;
    image_lights1->camera_id = CameraId::LONG_FOCUS;
    _preprocessor->_cached_lights_projections_array.push_back(image_lights1);

    std::shared_ptr<ImageLights> image_lights2(new ImageLights);
    image_lights2->timestamp = 0.7;
    image_lights2->camera_id = CameraId::LONG_FOCUS;
    _preprocessor->_cached_lights_projections_array.push_back(image_lights2);

    ASSERT_TRUE(_preprocessor->sync_image_with_cached_lights_projections(
        image,
        static_cast<CameraId>(cam_id),
        sync_time,
        data,
        &diff_image_pose_ts,
        &diff_image_sys_ts,
        &sync_ok));
    ASSERT_FALSE(sync_ok);
    EXPECT_DOUBLE_EQ(-0.2, diff_image_pose_ts);
    _preprocessor->_cached_lights_projections_array.clear();
    _preprocessor->_last_output_ts = 0.0;
  }

  cam_id = 0;
  diff_image_pose_ts = 0.0;
  sync_ok = false;
  {
    std::shared_ptr<ImageLights> image_lights1(new ImageLights);
    image_lights1->timestamp = -1.0;
    image_lights1->camera_id = CameraId::LONG_FOCUS;
    _preprocessor->_cached_lights_projections_array.push_back(image_lights1);

    std::shared_ptr<ImageLights> image_lights2(new ImageLights);
    image_lights2->timestamp = -0.3;
    image_lights2->camera_id = CameraId::LONG_FOCUS;
    _preprocessor->_cached_lights_projections_array.push_back(image_lights2);

    ASSERT_TRUE(_preprocessor->sync_image_with_cached_lights_projections(
        image,
        static_cast<CameraId>(cam_id),
        sync_time,
        data,
        &diff_image_pose_ts,
        &diff_image_sys_ts,
        &sync_ok));
    ASSERT_FALSE(sync_ok);
    EXPECT_DOUBLE_EQ(0.3, diff_image_pose_ts);
    _preprocessor->_cached_lights_projections_array.clear();
    _preprocessor->_last_output_ts = 0.0;
  }

  cam_id = 0;
  diff_image_pose_ts = 0.0;
  sync_ok = false;
  {
    std::shared_ptr<ImageLights> image_lights1(new ImageLights);
    image_lights1->timestamp = -1.0;
    image_lights1->camera_id = CameraId::LONG_FOCUS;
    _preprocessor->_cached_lights_projections_array.push_back(image_lights1);

    std::shared_ptr<ImageLights> image_lights2(new ImageLights);
    image_lights2->timestamp = -0.3;
    image_lights2->camera_id = CameraId::LONG_FOCUS;
    _preprocessor->_cached_lights_projections_array.push_back(image_lights2);

    // set _last_no_signals_ts, test query /tf in low frequence case
    _preprocessor->_last_no_signals_ts = -0.2;

    ASSERT_TRUE(_preprocessor->sync_image_with_cached_lights_projections(
        image,
        static_cast<CameraId>(cam_id),
        sync_time,
        data,
        &diff_image_pose_ts,
        &diff_image_sys_ts,
        &sync_ok));
    ASSERT_FALSE(sync_ok);
    EXPECT_DOUBLE_EQ(0.0, diff_image_pose_ts);
    _preprocessor->_cached_lights_projections_array.clear();
    _preprocessor->_last_output_ts = 0.0;
    _preprocessor->_last_no_signals_ts = 0.0;
  }
}

TEST_F(TLPreprocessorTest, test_setters_and_getters) {
  ASSERT_TRUE(_preprocessor->init());

  double no_signals_interval_seconds = 0.0;
  _preprocessor->set_no_signals_interval_seconds(5.0);
  _preprocessor->get_no_signals_interval_seconds(&no_signals_interval_seconds);
  EXPECT_DOUBLE_EQ(5.0, _preprocessor->_no_signals_interval_seconds);
  EXPECT_DOUBLE_EQ(5.0, no_signals_interval_seconds);

  _preprocessor->set_no_signals_interval_seconds(1000.0);
  _preprocessor->get_no_signals_interval_seconds(&no_signals_interval_seconds);
  EXPECT_DOUBLE_EQ(1000.0, _preprocessor->_no_signals_interval_seconds);
  EXPECT_DOUBLE_EQ(1000.0, no_signals_interval_seconds);

  _preprocessor->set_no_signals_interval_seconds(-23421.0);
  _preprocessor->get_no_signals_interval_seconds(&no_signals_interval_seconds);
  EXPECT_DOUBLE_EQ(-23421.0, _preprocessor->_no_signals_interval_seconds);
  EXPECT_DOUBLE_EQ(-23421.0, no_signals_interval_seconds);

}

TEST_F(TLPreprocessorTest, test_project_lights) {
  ASSERT_TRUE(_preprocessor->init());

  // empty signals
  {
    CarPose pose;
    std::vector<adu::common::hdmap::Signal> signals;
    MultiCamerasProjection projection;
    FLAGS_traffic_light_projection = "MultiCamerasProjection";

    std::shared_ptr<LightPtrs> lights_on_image;
    std::shared_ptr<LightPtrs> lights_outside_image;
    ASSERT_TRUE(projection.init());
    ASSERT_TRUE(_preprocessor->project_lights(
        projection,
        signals,
        pose,
        LONG_FOCUS,
        lights_on_image,
        lights_outside_image));
  }

  // invalid camera id
  {
    CarPose pose;
    std::vector<adu::common::hdmap::Signal> signals(1);
    MultiCamerasProjection projection;
    FLAGS_traffic_light_projection = "MultiCamerasProjection";

    std::shared_ptr<LightPtrs> lights_on_image;
    std::shared_ptr<LightPtrs> lights_outside_image;
    ASSERT_TRUE(projection.init());

    prepare_test_data_long_focus(&pose, &(signals[0]));

    int cam_id = -1;
    ASSERT_FALSE(_preprocessor->project_lights(
        projection,
        signals,
        pose,
        static_cast<CameraId>(cam_id),
        lights_on_image,
        lights_outside_image));
  }
  {
    CarPose pose;
    std::vector<adu::common::hdmap::Signal> signals(1);
    MultiCamerasProjection projection;
    FLAGS_traffic_light_projection = "MultiCamerasProjection";

    std::shared_ptr<LightPtrs> lights_on_image;
    std::shared_ptr<LightPtrs> lights_outside_image;
    ASSERT_TRUE(projection.init());

    prepare_test_data_long_focus(&pose, &(signals[0]));

    int cam_id = 1000;
    ASSERT_FALSE(_preprocessor->project_lights(
        projection,
        signals,
        pose,
        static_cast<CameraId>(cam_id),
        lights_on_image,
        lights_outside_image));
  }

  // long focus project on image
  {
    CarPose pose;
    std::vector<adu::common::hdmap::Signal> signals(1);
    MultiCamerasProjection projection;
    FLAGS_traffic_light_projection = "MultiCamerasProjection";

    std::shared_ptr<LightPtrs> lights_on_image(new LightPtrs);
    std::shared_ptr<LightPtrs> lights_outside_image(new LightPtrs);
    ASSERT_TRUE(projection.init());
    _preprocessor->set_camera_is_working_flag(LONG_FOCUS, true);
    prepare_test_data_long_focus(&pose, &(signals[0]));

    int cam_id = 0;
    ASSERT_TRUE(_preprocessor->project_lights(
        projection,
        signals,
        pose,
        static_cast<CameraId>(cam_id),
        lights_on_image,
        lights_outside_image));
    EXPECT_EQ(1, lights_on_image->size());
    EXPECT_EQ(0, lights_outside_image->size());
  }

  // short focus project on image
  {
    CarPose pose;
    std::vector<adu::common::hdmap::Signal> signals(1);
    MultiCamerasProjection projection;
    FLAGS_traffic_light_projection = "MultiCamerasProjection";

    std::shared_ptr<LightPtrs> lights_on_image(new LightPtrs);
    std::shared_ptr<LightPtrs> lights_outside_image(new LightPtrs);
    ASSERT_TRUE(projection.init());
    _preprocessor->set_camera_is_working_flag(SHORT_FOCUS, true);
    prepare_test_data_short_focus(&pose, &(signals[0]));

    ASSERT_TRUE(_preprocessor->project_lights(
        projection,
        signals,
        pose,
        SHORT_FOCUS,
        lights_on_image,
        lights_outside_image));
    EXPECT_EQ(1, lights_on_image->size());
    EXPECT_EQ(0, lights_outside_image->size());
  }

  // long focus project outside image
  {
    CarPose pose;
    std::vector<adu::common::hdmap::Signal> signals(1);
    MultiCamerasProjection projection;
    FLAGS_traffic_light_projection = "MultiCamerasProjection";

    std::shared_ptr<LightPtrs> lights_on_image(new LightPtrs);
    std::shared_ptr<LightPtrs> lights_outside_image(new LightPtrs);
    ASSERT_TRUE(projection.init());
    _preprocessor->set_camera_is_working_flag(LONG_FOCUS, true);
    prepare_test_data_long_focus(&pose, &(signals[0]));
    pose._pose(0, 3) = pose._pose(0, 3) + 100000;
    int cam_id = 0;
    ASSERT_TRUE(_preprocessor->project_lights(
        projection,
        signals,
        pose,
        static_cast<CameraId>(cam_id),
        lights_on_image,
        lights_outside_image));
    EXPECT_EQ(0, lights_on_image->size());
    EXPECT_EQ(1, lights_outside_image->size());
  }

  // short focus project outside image
  {
    CarPose pose;
    std::vector<adu::common::hdmap::Signal> signals(1);
    MultiCamerasProjection projection;
    FLAGS_traffic_light_projection = "MultiCamerasProjection";

    std::shared_ptr<LightPtrs> lights_on_image(new LightPtrs);
    std::shared_ptr<LightPtrs> lights_outside_image(new LightPtrs);
    ASSERT_TRUE(projection.init());
    _preprocessor->set_camera_is_working_flag(SHORT_FOCUS, true);
    prepare_test_data_short_focus(&pose, &(signals[0]));
    pose._pose(0, 3) = pose._pose(0, 3) + 100000;
    int cam_id = 0;
    ASSERT_TRUE(_preprocessor->project_lights(
        projection,
        signals,
        pose,
        static_cast<CameraId>(cam_id),
        lights_on_image,
        lights_outside_image));
    EXPECT_EQ(0, lights_on_image->size());
    EXPECT_EQ(1, lights_outside_image->size());
  }
}

TEST_F(TLPreprocessorTest, test_select_image) {
  ASSERT_TRUE(_preprocessor->init());
  std::map<int, int> image_borders_size = {
      {static_cast<int>(CameraId::LONG_FOCUS), 100},
      {static_cast<int>(CameraId::SHORT_FOCUS), 100},
      {static_cast<int>(CameraId::NARROW_FOCUS), 100},
      {static_cast<int>(CameraId::WIDE_FOCUS), 100}
  };

  {
    CarPose pose;
    int num_camera_ids = 2;
    CameraId selection = CameraId::UNKNOWN;
    std::vector<std::shared_ptr<LightPtrs> > lights_on_image_array(
        num_camera_ids);
    std::vector<std::shared_ptr<LightPtrs> > lights_outside_image_array(
        num_camera_ids);
    for (auto &light_ptrs : lights_on_image_array) {
      light_ptrs.reset(new LightPtrs);
    }
    for (auto &light_ptrs : lights_outside_image_array) {
      light_ptrs.reset(new LightPtrs);
    }

    _preprocessor->select_image(
        pose,
        lights_on_image_array,
        lights_outside_image_array,
        image_borders_size,
        &selection);
    EXPECT_EQ(CameraId::LONG_FOCUS, selection);
  }

  {
    CarPose pose;
    int num_camera_ids = 2;
    CameraId selection = CameraId::UNKNOWN;
    std::vector<std::shared_ptr<LightPtrs> > lights_on_image_array(
        num_camera_ids);
    std::vector<std::shared_ptr<LightPtrs> > lights_outside_image_array(
        num_camera_ids);
    for (auto &light_ptrs : lights_on_image_array) {
      light_ptrs.reset(new LightPtrs);
    }
    for (auto &light_ptrs : lights_outside_image_array) {
      light_ptrs.reset(new LightPtrs);
    }

    MultiCamerasProjection projection;
    FLAGS_traffic_light_projection = "MultiCamerasProjection";
    ASSERT_TRUE(projection.init());

    std::vector<adu::common::hdmap::Signal> signals(1);
    prepare_test_data_long_focus(&pose, &(signals[0]));
    int cam_id = 0;
    ASSERT_TRUE(_preprocessor->project_lights(
        projection,
        signals,
        pose,
        static_cast<CameraId>(cam_id),
        lights_on_image_array[0],
        lights_outside_image_array[0]));

    ASSERT_TRUE(_preprocessor->set_camera_is_working_flag(LONG_FOCUS, true));
    _preprocessor->select_image(
        pose,
        lights_on_image_array,
        lights_outside_image_array,
        image_borders_size,
        &selection);
    EXPECT_EQ(CameraId::LONG_FOCUS, selection);

    ASSERT_TRUE(_preprocessor->set_camera_is_working_flag(LONG_FOCUS, false));
    _preprocessor->select_image(
        pose,
        lights_on_image_array,
        lights_outside_image_array,
        image_borders_size,
        &selection);
    EXPECT_EQ(CameraId::LONG_FOCUS, selection);
  }

  // test project on image border region
  {
    CarPose pose;
    int num_camera_ids = 4;
    CameraId selection = CameraId::UNKNOWN;
    std::vector<std::shared_ptr<LightPtrs> > lights_on_image_array(
        num_camera_ids);
    std::vector<std::shared_ptr<LightPtrs> > lights_outside_image_array(
        num_camera_ids);
    for (auto &light_ptrs : lights_on_image_array) {
      light_ptrs.reset(new LightPtrs);
    }
    for (auto &light_ptrs : lights_outside_image_array) {
      light_ptrs.reset(new LightPtrs);
    }

    MultiCamerasProjection projection;
    FLAGS_traffic_light_projection = "MultiCamerasProjection";
    ASSERT_TRUE(projection.init());

    std::vector<adu::common::hdmap::Signal> signals(1);
    prepare_test_data_long_focus(&pose, &(signals[0]));

    int cam_id = 0;
    ASSERT_TRUE(_preprocessor->set_camera_is_working_flag(LONG_FOCUS, true));
    ASSERT_TRUE(_preprocessor->set_camera_is_working_flag(SHORT_FOCUS, true));
    ASSERT_TRUE(_preprocessor->project_lights(
        projection,
        signals,
        pose,
        static_cast<CameraId>(cam_id),
        lights_on_image_array[0],
        lights_outside_image_array[0]));

    _preprocessor->select_image(
        pose,
        lights_on_image_array,
        lights_outside_image_array,
        image_borders_size,
        &selection);
    EXPECT_EQ(CameraId::LONG_FOCUS, selection);

    image_borders_size[static_cast<int>(CameraId::LONG_FOCUS)] = 500;
    image_borders_size[static_cast<int>(CameraId::SHORT_FOCUS)] = 500;
    ASSERT_TRUE(_preprocessor->set_camera_is_working_flag(WIDE_FOCUS, true));
    _preprocessor->select_image(
        pose,
        lights_on_image_array,
        lights_outside_image_array,
        image_borders_size,
        &selection);
    EXPECT_EQ(CameraId::SHORT_FOCUS, selection);
    image_borders_size[static_cast<int>(CameraId::LONG_FOCUS)] = 100;
    image_borders_size[static_cast<int>(CameraId::SHORT_FOCUS)] = 100;
  }
}

TEST_F(TLPreprocessorTest, test_sync_image) {
  ASSERT_TRUE(_preprocessor->init());
  std::shared_ptr<Image> image(new Image);
  int cam_id = -1;
  double sync_time = 0.0;
  std::shared_ptr<ImageLights> data(new ImageLights);
  bool should_pub = false;

  ASSERT_FALSE(_preprocessor->sync_image(
      image,
      sync_time,
      static_cast<CameraId>(cam_id),
      &data,
      &should_pub));
  ASSERT_FALSE(should_pub);

  cam_id = 1000;
  ASSERT_FALSE(_preprocessor->sync_image(
      image,
      sync_time,
      static_cast<CameraId>(cam_id),
      &data,
      &should_pub));
  ASSERT_FALSE(should_pub);

  cam_id = 0;
  ASSERT_TRUE(_preprocessor->sync_image(
      image,
      sync_time,
      static_cast<CameraId>(cam_id),
      &data,
      &should_pub));
  ASSERT_TRUE(should_pub);

  cam_id = 1;
  should_pub = true;
  sync_time = 0.2;
  {
    std::shared_ptr<ImageLights> image_lights(new ImageLights);
    image_lights->timestamp = 0.2;
    image_lights->camera_id = CameraId::LONG_FOCUS;
    _preprocessor->_cached_lights_projections_array.push_back(image_lights);
    ASSERT_FALSE(_preprocessor->sync_image(
        image,
        sync_time,
        static_cast<CameraId>(cam_id),
        &data,
        &should_pub));
    ASSERT_FALSE(should_pub);
    _preprocessor->_cached_lights_projections_array.clear();
  }

  cam_id = 0;
  should_pub = false;
  sync_time = 0.0;
  {
    std::shared_ptr<ImageLights> image_lights(new ImageLights);
    image_lights->timestamp = 0.01;
    image_lights->camera_id = CameraId::LONG_FOCUS;
    _preprocessor->_cached_lights_projections_array.push_back(image_lights);
    ASSERT_TRUE(_preprocessor->sync_image(
        image,
        sync_time,
        static_cast<CameraId>(cam_id),
        &data,
        &should_pub));
    ASSERT_TRUE(should_pub);
    _preprocessor->_cached_lights_projections_array.clear();
  }

  cam_id = 0;
  should_pub = false;
  sync_time = 0.0;
  {
    std::shared_ptr<ImageLights> image_lights(new ImageLights);
    image_lights->timestamp = 0.01;
    image_lights->camera_id = CameraId::SHORT_FOCUS;
    _preprocessor->_cached_lights_projections_array.push_back(image_lights);
    _preprocessor->_cached_signal_nums_array.push_back(std::make_pair(0.00, 1));
    ASSERT_TRUE(_preprocessor->sync_image(
        image,
        sync_time,
        static_cast<CameraId>(cam_id),
        &data,
        &should_pub));
    ASSERT_TRUE(should_pub);
    EXPECT_EQ(data->num_signals, 0);
    _preprocessor->_cached_lights_projections_array.clear();
    _preprocessor->_cached_signal_nums_array.clear();
  }

  cam_id = 0;
  should_pub = false;
  sync_time = 0.1;
  {
    std::shared_ptr<ImageLights> image_lights(new ImageLights);
    image_lights->timestamp = 0.01;
    image_lights->camera_id = CameraId::SHORT_FOCUS;
    _preprocessor->_cached_lights_projections_array.push_back(image_lights);
    _preprocessor->_cached_signal_nums_array.push_back(std::make_pair(0.01, 1));
    ASSERT_TRUE(_preprocessor->sync_image(
        image,
        sync_time,
        static_cast<CameraId>(cam_id),
        &data,
        &should_pub));
    ASSERT_TRUE(should_pub);
    EXPECT_EQ(data->num_signals, 1);

    ASSERT_TRUE(_preprocessor->sync_image(
        image,
        sync_time,
        static_cast<CameraId>(cam_id),
        &data,
        &should_pub));
    ASSERT_TRUE(should_pub);
    EXPECT_EQ(data->num_signals, 1);

    _preprocessor->_cached_lights_projections_array.clear();
    _preprocessor->_cached_signal_nums_array.clear();
  }

  cam_id = 0;
  should_pub = false;
  sync_time = 0.1;
  {
    std::shared_ptr<ImageLights> image_lights(new ImageLights);
    image_lights->timestamp = 0.01;
    image_lights->camera_id = CameraId::SHORT_FOCUS;
    _preprocessor->_cached_lights_projections_array.push_back(image_lights);
    _preprocessor->_cached_signal_nums_array.push_back(std::make_pair(-0.5, 1));
    ASSERT_TRUE(_preprocessor->sync_image(
        image,
        sync_time,
        static_cast<CameraId>(cam_id),
        &data,
        &should_pub));
    ASSERT_TRUE(should_pub);
    EXPECT_EQ(data->num_signals, 0);

    _preprocessor->_cached_lights_projections_array.clear();
    _preprocessor->_cached_signal_nums_array.clear();
  }
}

TEST_F(TLPreprocessorTest, test_get_max_min_focal_len_camera_id) {
  config_manager::fLS::FLAGS_config_manager_path = "conf/config_manager.config";
  ASSERT_TRUE(_preprocessor->init());

  // test get_max_focal_len_camera_id()
  EXPECT_EQ(static_cast<int>(CameraId::LONG_FOCUS),
            _preprocessor->get_max_focal_len_camera_id());

  ASSERT_TRUE(_preprocessor->set_camera_is_working_flag(CameraId::LONG_FOCUS, true));
  EXPECT_EQ(static_cast<int>(CameraId::LONG_FOCUS),
            _preprocessor->get_max_focal_len_camera_id());

  ASSERT_TRUE(_preprocessor->set_camera_is_working_flag(CameraId::LONG_FOCUS, false));
  ASSERT_TRUE(_preprocessor->set_camera_is_working_flag(CameraId::NARROW_FOCUS, true));
  EXPECT_EQ(static_cast<int>(CameraId::NARROW_FOCUS),
            _preprocessor->get_max_focal_len_camera_id());

  ASSERT_TRUE(_preprocessor->set_camera_is_working_flag(CameraId::NARROW_FOCUS, false));
  ASSERT_TRUE(_preprocessor->set_camera_is_working_flag(CameraId::SHORT_FOCUS, true));
  EXPECT_EQ(static_cast<int>(CameraId::SHORT_FOCUS),
            _preprocessor->get_max_focal_len_camera_id());

  ASSERT_TRUE(_preprocessor->set_camera_is_working_flag(CameraId::SHORT_FOCUS, false));
  ASSERT_TRUE(_preprocessor->set_camera_is_working_flag(CameraId::WIDE_FOCUS, true));
  EXPECT_EQ(static_cast<int>(CameraId::WIDE_FOCUS),
            _preprocessor->get_max_focal_len_camera_id());

  // test get_min_focal_len_camera_id()
  EXPECT_EQ(static_cast<int>(CameraId::WIDE_FOCUS),
            _preprocessor->get_min_focal_len_camera_id());

  ASSERT_TRUE(_preprocessor->set_camera_is_working_flag(CameraId::WIDE_FOCUS, true));
  EXPECT_EQ(static_cast<int>(CameraId::WIDE_FOCUS),
            _preprocessor->get_min_focal_len_camera_id());

  ASSERT_TRUE(_preprocessor->set_camera_is_working_flag(CameraId::WIDE_FOCUS, false));
  ASSERT_TRUE(_preprocessor->set_camera_is_working_flag(CameraId::SHORT_FOCUS, true));
  EXPECT_EQ(static_cast<int>(CameraId::SHORT_FOCUS),
            _preprocessor->get_min_focal_len_camera_id());

  ASSERT_TRUE(_preprocessor->set_camera_is_working_flag(CameraId::SHORT_FOCUS, false));
  ASSERT_TRUE(_preprocessor->set_camera_is_working_flag(CameraId::NARROW_FOCUS, true));
  EXPECT_EQ(static_cast<int>(CameraId::NARROW_FOCUS),
            _preprocessor->get_min_focal_len_camera_id());

  ASSERT_TRUE(_preprocessor->set_camera_is_working_flag(CameraId::NARROW_FOCUS, false));
  ASSERT_TRUE(_preprocessor->set_camera_is_working_flag(CameraId::LONG_FOCUS, true));
  EXPECT_EQ(static_cast<int>(CameraId::LONG_FOCUS),
            _preprocessor->get_min_focal_len_camera_id());
}

TEST_F(TLPreprocessorTest, test_set_get_max_cached_image_lights_array_size) {
  config_manager::fLS::FLAGS_config_manager_path = "conf/config_manager.config";
  ASSERT_TRUE(_preprocessor->init());

  size_t max_cached_image_lights_array_size = 0;
  ASSERT_TRUE(_preprocessor
                  ->get_max_cached_image_lights_array_size(&max_cached_image_lights_array_size));
  ASSERT_TRUE(max_cached_image_lights_array_size == 100);

  ASSERT_TRUE(_preprocessor->set_max_cached_image_lights_array_size(200));
  ASSERT_TRUE(_preprocessor
                  ->get_max_cached_image_lights_array_size(&max_cached_image_lights_array_size));
  ASSERT_TRUE(max_cached_image_lights_array_size == 200);

  ASSERT_FALSE(_preprocessor->set_max_cached_image_lights_array_size(0));
}

}  // namespace traffic_light
}  // namespace perception
}  // namespace adu
