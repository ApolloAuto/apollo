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

#include "modules/drivers/lidar/hesai/parser/hesai_convert_component.h"

namespace apollo {
namespace drivers {
namespace hesai {

using apollo::cyber::Component;

bool HesaiConvertComponent::Init() {
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

  if (!parser_->Init()) {
    return false;
  }
  AINFO << "HesaiConvertComponent init success";
  return true;
}

bool HesaiConvertComponent::Proc(const std::shared_ptr<HesaiScan>& scan) {
  return parser_->Parse(scan);
}

}  // namespace hesai
}  // namespace drivers
}  // namespace apollo
