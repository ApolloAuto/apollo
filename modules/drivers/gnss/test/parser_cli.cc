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

// A command-line interface (CLI) tool of parser. It parses a binary log file.
// It is supposed to be
// used for verifying if the parser works properly.

#include <fstream>
#include <iostream>
#include <memory>

#include "modules/drivers/gnss/proto/config.pb.h"

#include "cyber/cyber.h"
#include "cyber/init.h"
#include "cyber/record/record_reader.h"
#include "modules/drivers/gnss/parser/data_parser.h"
#include "modules/drivers/gnss/stream/stream.h"

namespace apollo {
namespace drivers {
namespace gnss {

constexpr size_t BUFFER_SIZE = 128;

void ParseBin(const char* filename, DataParser* parser) {
  std::ios::sync_with_stdio(false);
  std::ifstream f(filename, std::ifstream::binary);
  char b[BUFFER_SIZE];
  while (f && cyber::OK()) {
    f.read(b, BUFFER_SIZE);
    std::string msg(reinterpret_cast<const char*>(b), f.gcount());
    parser->ParseRawData(msg);
  }
}

void ParseRecord(const char* filename, DataParser* parser) {
  cyber::record::RecordReader reader(filename);
  cyber::record::RecordMessage message;
  while (reader.ReadMessage(&message) && cyber::OK()) {
    if (message.channel_name == "/apollo/sensor/gnss/raw_data") {
      apollo::drivers::gnss::RawData msg;
      msg.ParseFromString(message.content);
      parser->ParseRawData(msg.data());
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

void Parse(const char* filename, const char* file_type,
           const std::shared_ptr<::apollo::cyber::Node>& node) {
  std::string type = std::string(file_type);
  config::Config config;
  if (!apollo::cyber::common::GetProtoFromFile(
          std::string("/apollo/modules/drivers/gnss/conf/gnss_conf.pb.txt"),
          &config)) {
    std::cout << "Unable to load gnss conf file";
  }
  DataParser* parser = new DataParser(config, node);
  parser->Init();
  if (type == "bin") {
    ParseBin(filename, parser);
  } else if (type == "record") {
    ParseRecord(filename, parser);
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
    std::cout << "Usage: " << argv[0] << " filename [record|bin]" << std::endl;
    return 0;
  }
  ::apollo::cyber::Init("parser_cli");
  std::shared_ptr<::apollo::cyber::Node> parser_node(
      ::apollo::cyber::CreateNode("parser_cli"));
  ::apollo::drivers::gnss::Parse(argv[1], argv[2], parser_node);
  return 0;
}
