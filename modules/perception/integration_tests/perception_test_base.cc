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

#include "modules/perception/integration_tests/perception_test_base.h"

#include <cstdlib>

#include "pcl/io/pcd_io.h"
#include "pcl_conversions/pcl_conversions.h"

#include "google/protobuf/io/zero_copy_stream_impl.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/perception/common/pcl_types.h"

namespace apollo {
namespace perception {

using common::adapter::AdapterManager;

DEFINE_string(test_data_dir, "", "the test data folder");
DEFINE_string(test_pointcloud_file, "", "The pointcloud file used in test");
DEFINE_string(test_localization_file, "", "The localization test file");
DEFINE_string(test_chassis_file, "", "The chassis test file");
DEFINE_string(perception_config_file, "", "The perception config file");

void PerceptionTestBase::SetUpTestCase() {
  FLAGS_work_root = "modules/perception";

  FLAGS_perception_config_file =
      "modules/perception/integration_tests/testdata/conf/"
      "perception_config.pb.txt";
  FLAGS_perception_adapter_config_filename =
      "modules/perception/integration_tests/testdata/conf/adapter.conf";

  FLAGS_dag_config_path =
      "./integration_tests/testdata/conf/dag_streaming.config";

  FLAGS_test_pointcloud_file =
      "modules/perception/integration_tests/testdata/point_cloud_test_file.pcd";

  FLAGS_is_serialize_point_cloud = false;
  FLAGS_map_radius = 60.0;
  FLAGS_map_sample_step = 1;

  FLAGS_enable_hdmap_input = true;
  FLAGS_onboard_roi_filter = "HdmapROIFilter";
  FLAGS_onboard_segmentor = "CNNSegmentation";
  FLAGS_onboard_object_filter = "LowObjectFilter";
  FLAGS_onboard_object_builder = "MinBoxObjectBuilder";
  FLAGS_onboard_tracker = "HmObjectTracker";
  FLAGS_onboard_type_fuser = "DummyTypeFuser";
  FLAGS_obstacle_module_name = "perception_obstacle";
  FLAGS_tf2_buff_in_ms = 20;
  FLAGS_lidar_tf2_frame_id = "novatel";
  FLAGS_lidar_tf2_child_frame_id = "velodyne64";
  FLAGS_enable_visualization = false;
  FLAGS_onboard_radar_detector = "ModestRadarDetector";
  FLAGS_front_radar_forward_distance = 100;
  FLAGS_localization_buffer_size = 40;
  FLAGS_radar_tf2_frame_id = "novatel";
  FLAGS_radar_tf2_child_frame_id = "radar";
  FLAGS_onboard_fusion = "ProbabilisticFusion";
  FLAGS_light_height_adjust = 0;
  FLAGS_traffic_light_rectifier = "UnityRectify";
  FLAGS_traffic_light_recognizer = "UnityRecognize";
  FLAGS_traffic_light_reviser = "ColorReviser";
  FLAGS_query_signal_range = 200.0;
  FLAGS_output_debug_img = false;
  FLAGS_output_raw_img = false;
  FLAGS_q_matrix_coefficient_amplifier = 0.5;
  // FLAGS_r_matrix_amplifer = 1;
  // FLAGS_p_matrix_amplifer = 1;
  FLAGS_a_matrix_covariance_coeffcient_1 = 0.05;
  FLAGS_a_matrix_covariance_coeffcient_2 = 0.05;
  FLAGS_test_localization_file = "";
  FLAGS_test_chassis_file = "";
}

#define FEED_DATA_TO_ADAPTER(TYPE, DATA)                              \
  if (!AdapterManager::Get##TYPE()) {                                 \
    AERROR << #TYPE                                                   \
        " is not registered in adapter manager, check adapter file "; \
    return false;                                                     \
  }                                                                   \
  AdapterManager::Feed##TYPE##Data(DATA);

#define FEED_FILE_TO_ADAPTER(TYPE, FILENAME)                                   \
  if (!AdapterManager::Get##TYPE()) {                                          \
    AERROR << #TYPE                                                            \
        " is not registered in adapter manager, check adapter file "           \
           << FLAGS_perception_adapter_config_filename;                        \
    return false;                                                              \
  }                                                                            \
  if (!FILENAME.empty()) {                                                     \
    if (!AdapterManager::Feed##TYPE##File(FLAGS_test_data_dir + "/" +          \
                                          FILENAME)) {                         \
      AERROR << "Failed to feed " #TYPE " file " << FLAGS_test_data_dir << "/" \
             << FILENAME;                                                      \
      return false;                                                            \
    }                                                                          \
    AINFO << "Using " #TYPE << " provided by " << FLAGS_test_data_dir << "/"   \
          << FILENAME;                                                         \
  }

bool PerceptionTestBase::SetUpAdapters() {
  if (!AdapterManager::Initialized()) {
    AdapterManager::Init(FLAGS_perception_adapter_config_filename);
  }

  FEED_DATA_TO_ADAPTER(PointCloud, cloud_blob_);
  FEED_FILE_TO_ADAPTER(Localization, FLAGS_test_localization_file);
  FEED_FILE_TO_ADAPTER(Chassis, FLAGS_test_chassis_file);
  return true;
}

void PerceptionTestBase::SetUp() {
  SetUpTestCase();

  // load PCD file and transfer to point cloud
  pcl::PCLPointCloud2 pcl_pointcloud;
  CHECK_NE(-1, pcl::io::loadPCDFile(FLAGS_test_pointcloud_file, pcl_pointcloud))
      << FLAGS_test_pointcloud_file;

  pcl_conversions::fromPCL(pcl_pointcloud, cloud_blob_);

  CHECK(SetUpAdapters()) << "Failed to setup adapters";
  perception_.reset(new Perception);
  CHECK(perception_->Init().ok()) << "Failed to init perception module";
}

void PerceptionTestBase::UpdateData() {
  CHECK(SetUpAdapters()) << "Failed to setup adapters";
}

bool PerceptionTestBase::RunPerception(const std::string& test_case_name,
                                       int case_num) {
  if (perception_->Start() != common::Status::OK()) {
    AERROR << "Failed to start perception.";
    return false;
  }
  perception_->Stop();
  // TODO(All): Finish implementation of RunPerception here.
  return true;
}

}  // namespace perception
}  // namespace apollo
