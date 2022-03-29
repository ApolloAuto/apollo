/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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
#include "modules/drivers/lidar/lidar_robosense/parser/robosense_parser_factory.h"

#include "cyber/cyber.h"
#include "modules/drivers/lidar/lidar_robosense/parser/robosense16_parser.h"
#include "modules/drivers/lidar/lidar_robosense/parser/robosense_parser.h"

namespace apollo {
namespace drivers {
namespace robosense {

RobosenseParser* RobosenseParserFactory::create_parser(
    const apollo::drivers::suteng::SutengConfig& config) {
  if (config.model() == apollo::drivers::suteng::VLP16) {
    return new Robosense16Parser(config);
  } else {
    AERROR << " invalid model, must be VLP16";
    return nullptr;
  }
}

}  // namespace robosense
}  // namespace drivers
}  // namespace apollo
