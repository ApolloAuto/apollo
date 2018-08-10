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

#include "modules/monitor/sysmon/base/metric_data_single.h"

namespace apollo {
namespace monitor {
namespace sysmon {

DCL_SYSMON_PLUGIN_API_VER(PLUGIN_API_VERSION_1);

DCL_METRIC_DATADES_CREATOR(SingleIntMetricDataDes);

DCL_METRIC_DATADES_CREATOR(PresenceMetricDataDes);

DCL_METRIC_DATADES_CREATOR(TemperatureMetricDataDes);

DCL_METRIC_DATADES_CREATOR(OnOffStatusMetricDataDes);

DCL_METRIC_DATADES_LIST({
  {"single_int", "single integer data deserializer",
      _SYSMON_TYPE_TO_CREATOR_NAME(SingleIntMetricDataDes)},
  {"presence", "presence status data deserializer",
      _SYSMON_TYPE_TO_CREATOR_NAME(PresenceMetricDataDes)},
  {"temperature", "temperature data deserializer",
      _SYSMON_TYPE_TO_CREATOR_NAME(PresenceMetricDataDes)},
  {"status", "on-off status data deserializer",
      _SYSMON_TYPE_TO_CREATOR_NAME(OnOffStatusMetricDataDes)},
  {nullptr, nullptr, nullptr}
});


}  // namespace sysmon
}  // namespace monitor
}  // namespace apollo
