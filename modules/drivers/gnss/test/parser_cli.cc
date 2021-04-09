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

// A command-line interface (CLI) tool of parser. It parses a binary log file.
// It is supposed to be
// used for verifying if the parser works properly.

#include <ros/ros.h>
#include <fstream>
#include <iostream>
#include <memory>

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/util/file.h"
#include "modules/drivers/gnss/gnss_gflags.h"
#include "modules/drivers/gnss/parser/data_parser.h"
#include "modules/drivers/gnss/proto/config.pb.h"
#include "modules/drivers/gnss/stream/stream.h"
#include "ros/include/rosbag/bag.h"
#include "ros/include/rosbag/view.h"

namespace apollo {
namespace drivers {
namespace gnss {

constexpr size_t BUFFER_SIZE = 128;

void ParseBin(const char* filename, DataParser* parser) {
  std::ios::sync_with_stdio(false);
  std::ifstream f(filename, std::ifstream::binary);
  char b[BUFFER_SIZE];
  while (f) {
    f.read(b, BUFFER_SIZE);
    std_msgs::StringPtr msg(new std_msgs::String);
    msg->data.assign(reinterpret_cast<const char*>(b), f.gcount());
    parser->ParseRawData(msg);
  }
}

void ParseBag(const char* filename, DataParser* parser) {
  rosbag::Bag bag;
  bag.open(filename, rosbag::bagmode::Read);

  std::vector<std::string> topics = {"/apollo/sensor/gnss/raw_data"};
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  for (auto const m : view) {
    std_msgs::String::ConstPtr msg = m.instantiate<std_msgs::String>();
    if (msg != nullptr) {
      parser->ParseRawData(msg);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

void Parse(const char* filename, const char* file_type) {
  std::string type = std::string(file_type);
  config::Config config;
  if (!common::util::GetProtoFromFile(
          std::string("/apollo/modules/drivers/gnss/conf/gnss_conf.pb.txt"),
          &config)) {
    std::cout << "Unable to load gnss conf file";
  }
  DataParser* parser = new DataParser(config);
  parser->Init();
  if (type == "bag") {
    ParseBag(filename, parser);
  } else if (type == "bin") {
    ParseBin(filename, parser);
  } else {
    std::cout << "unknown file type";
  }
  delete parser;
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cout << "Usage: " << argv[0] << " filename [bag|bin]" << std::endl;
    return 0;
  }

  ros::init(argc, argv, "parser_cli");
  ros::Time::init();
  ros::Time::now();
  apollo::common::adapter::AdapterManager::Init(FLAGS_adapter_config_filename);
  ::apollo::drivers::gnss::Parse(argv[1], argv[2]);
  return 0;
}
