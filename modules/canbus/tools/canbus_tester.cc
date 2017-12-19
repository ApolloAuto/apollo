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

#include "gflags/gflags.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/log.h"
#include "ros/include/ros/ros.h"
#include "std_msgs/String.h"

#include "modules/canbus/common/canbus_gflags.h"
#include "modules/common/util/file.h"
#include "modules/control/proto/control_cmd.pb.h"

int main(int32_t argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_alsologtostderr = true;

  ros::init(argc, argv, "canbus_tester");

  ros::NodeHandle nh;
  ros::Publisher pub =
      nh.advertise<std_msgs::String>(FLAGS_control_command_topic, 100);

  apollo::control::ControlCommand control_cmd;
  if (!apollo::common::util::GetProtoFromFile(FLAGS_canbus_test_file,
                                              &control_cmd)) {
    AERROR << "failed to load file: " << FLAGS_canbus_test_file;
    return -1;
  }

  std_msgs::String msg;
  control_cmd.SerializeToString(&msg.data);
  ros::Rate loop_rate(1);  // frequency

  while (ros::ok()) {
    pub.publish(msg);
    ros::spinOnce();  // yield
    loop_rate.sleep();
  }

  return 0;
}
