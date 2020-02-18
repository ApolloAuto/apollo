/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#ifndef LIDAR_HESAI_HESAI_ALL_COMPONENT_H_
#define LIDAR_HESAI_HESAI_ALL_COMPONENT_H_

#include <list>
#include <memory>
#include <string>
#include <thread>

#include "cyber/cyber.h"
#include "modules/drivers/hesai/const_var.h"
#include "modules/drivers/hesai/driver.h"
#include "modules/drivers/hesai/parser.h"
#include "modules/drivers/hesai/type_defs.h"
#include "modules/drivers/hesai/proto/config.pb.h"

namespace apollo {
namespace drivers {
namespace hesai {

using apollo::cyber::Component;

class HesaiComponent : public Component<> {
 public:
  ~HesaiComponent() {}
  bool Init() override {
    if (!GetProtoConfig(&conf_)) {
      AERROR << "load config error, file:" << config_file_path_;
      return false;
    }

    AINFO << "conf:" << conf_.DebugString();
    Parser* parser = ParserFactory::CreateParser(node_, conf_);
    if (parser == nullptr) {
      AERROR << "create parser error";
      return false;
    }
    parser_.reset(parser);
    driver_.reset(new HesaiDriver(node_, conf_, parser_));

    if (!driver_->Init()) {
      AERROR << "driver init error";
      return false;
    }
    return true;
  }

 private:
  std::shared_ptr<HesaiDriver> driver_;
  std::shared_ptr<Parser> parser_;
  Config conf_;
};

CYBER_REGISTER_COMPONENT(HesaiComponent)

}  // namespace hesai
}  // namespace drivers
}  // namespace apollo

#endif
