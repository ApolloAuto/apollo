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

#include <ros/package.h>
#include "velodyne_unit_test.h"

namespace apollo {
namespace drivers {
namespace velodyne {

// conifg for different device
apollo::drivers::velodyne::Config _config_64;
apollo::drivers::velodyne::Config _config_32;
apollo::drivers::velodyne::Config _config_16;

// the first three scan messages subscribe from
// /dusensor/velodyne/packets(replay bag)
velodyne_msgs::VelodyneScanUnified scan_msg_2;
velodyne_msgs::VelodyneScanUnified scan_msg_3;

// the expect output of VelodyneParser::generate_pointcloud
VPointCloud expect_out_msg_2;
VPointCloud expect_out_msg_3;

// the expect output of VelodyneParser::order
VPointCloud expect_ordered_out_msg_2;
VPointCloud expect_ordered_out_msg_3;

// the actrually output message pointers
VPointCloud::Ptr out_msg_ptr_2(new VPointCloud());
VPointCloud::Ptr out_msg_ptr_3(new VPointCloud());

VelodyneParser* _parser;

void init_test_data() {
  std::string pkg_path = ros::package::getPath("velodyne_pointcloud");
  std::string msg_folder = pkg_path + "/tests/velodyne_test_msg_files/";

  // config for 64E_S2
  _config_64.calibration_file = pkg_path + "/params/64e_baidu_clear.yaml";
  _config_64.max_range = 70.0f;
  _config_64.min_range = 5.0f;
  _config_64.view_direction = 0.0;
  _config_64.view_width = M_PI * 2.0;
  _config_64.model = "64E_S2";
  _config_64.organized = true;
  _config_64.time_zone = 8;

  _parser = VelodyneParserFactory::create_parser(_config_64);
  _parser->setup();

  load_msg<velodyne_msgs::VelodyneScanUnified>(
      msg_folder + "scan_msg_43367.msg", &scan_msg_2);
  load_msg<velodyne_msgs::VelodyneScanUnified>(
      msg_folder + "scan_msg_43368.msg", &scan_msg_3);

  // the expect output of RawData::generate_pointcloud
  load_msg<VPointCloud>(msg_folder + "out_msg_43367.msg", &expect_out_msg_2);
  load_msg<VPointCloud>(msg_folder + "out_msg_43368_new.msg",
                        &expect_out_msg_3);

  // the expect output of RawData::order
  load_msg<VPointCloud>(msg_folder + "ordered_out_msg_43367.msg",
                        &expect_ordered_out_msg_2);
  load_msg<VPointCloud>(msg_folder + "ordered_out_msg_43368_new.msg",
                        &expect_ordered_out_msg_3);
  _parser->generate_pointcloud(
      boost::make_shared<velodyne_msgs::VelodyneScanUnified const>(scan_msg_2),
      out_msg_ptr_2);
}

TEST(TestSuiteVelodyneParser, generate_pointcloud) {
  _parser->generate_pointcloud(
      boost::make_shared<velodyne_msgs::VelodyneScanUnified const>(scan_msg_3),
      out_msg_ptr_3);
  compare_out_msg(expect_out_msg_3, *out_msg_ptr_3);
}

TEST(TestSuiteVelodyneParser, order) {
  _parser->order(out_msg_ptr_3);
  compare_out_msg(expect_ordered_out_msg_3, *out_msg_ptr_3);
}

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  apollo::drivers::velodyne::init_test_data();
  return RUN_ALL_TESTS();
}
