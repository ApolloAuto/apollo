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

#ifndef MODULES_LOCALIZATION_MSF_LOCAL_TOOL_BASE_EXPORTER_H
#define MODULES_LOCALIZATION_MSF_LOCAL_TOOL_BASE_EXPORTER_H

#include <rosbag/bag.h>
#include <memory>

namespace apollo {
namespace localization {
namespace msf {

/**
 * @class BaseExporter
 * @brief The base class for other exporter classes.
 */
class BaseExporter {
 public:
  typedef void (BaseExporter::*OnRosmsgCallback)(
      const rosbag::MessageInstance &msg);

  typedef std::shared_ptr<BaseExporter> Ptr;
  typedef std::shared_ptr<BaseExporter const> ConstPtr;

 protected:
  BaseExporter() {}

  ~BaseExporter() {}
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_MSF_LOCAL_TOOL_BASE_EXPORTER_H
