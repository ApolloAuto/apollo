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

#include "velodyne_unit_test.h"

namespace apollo {
namespace drivers {
namespace velodyne {

std::string _run_save_folder;
std::string _expected_save_folder;
// file name prefix
std::string _run_file_prefix;
std::string _expected_file_prefix;
int _start_seq;
int _end_seq;
int _wait_time;

TEST(TestSuiteVerifyPointCloud, verify_pointcloud) {
  for (int i = _start_seq; i <= _end_seq; i++) {
    std::string run_file = _run_save_folder + "/" + _run_file_prefix +
                           boost::lexical_cast<std::string>(i) + ".msg";
    std::cout << run_file << std::endl;

    std::string expected_file = _expected_save_folder + "/" +
                                _expected_file_prefix +
                                boost::lexical_cast<std::string>(i) + ".msg";
    std::cout << expected_file << std::endl;

    VPointCloud run_out_msg;
    load_msg<VPointCloud>(run_file, &run_out_msg);
    VPointCloud expected_out_msg;
    load_msg<VPointCloud>(expected_file, &expected_out_msg);

    compare_out_msg(expected_out_msg, run_out_msg);
  }
  remove_tmp_files(_run_save_folder, "64e_pointcloud_out");
}

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo

int main(int argc, char** argv) {
  ros::init(argc, argv, "verfiy_pointcloud");
  testing::InitGoogleTest(&argc, argv);
  ros::NodeHandle priv_nh("~");
  priv_nh.param("run_save_folder", apollo::drivers::velodyne::_run_save_folder,
                std::string(""));
  priv_nh.param("expected_save_folder",
                apollo::drivers::velodyne::_expected_save_folder,
                std::string(""));
  priv_nh.param("run_file_prefix", apollo::drivers::velodyne::_run_file_prefix,
                std::string(""));
  priv_nh.param("expected_file_prefix",
                apollo::drivers::velodyne::_expected_file_prefix,
                std::string(""));
  priv_nh.param("start_seq", apollo::drivers::velodyne::_start_seq, 0);
  priv_nh.param("end_seq", apollo::drivers::velodyne::_end_seq, 0);
  priv_nh.param("wait_time", apollo::drivers::velodyne::_wait_time, 0);

  ros::Duration(apollo::drivers::velodyne::_wait_time).sleep();
  ros::shutdown();

  return RUN_ALL_TESTS();
}
