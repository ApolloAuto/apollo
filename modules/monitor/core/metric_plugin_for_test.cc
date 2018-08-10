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

#include <stdio.h>

#include "modules/monitor/sysmon/base/metric_data_single.h"
#include "modules/monitor/sysmon/base/plugin_interface.h"

namespace apollo {
namespace monitor {
namespace sysmon {

DCL_SYSMON_PLUGIN_API_VER(PLUGIN_API_VERSION_1);

class PresenceMetric : public MetricQuerentInterface {
 public:
  PresenceMetric() : data_(0) {}

  MetricQueryResult query(MetricDataInterface **m_data, std::string *err)
      override {
    data_ = 1;
    *m_data = new PresenceMetricData(data_);
    return *m_data ? MetricQueryResult::OK : MetricQueryResult::ERROR;
  }

 private:
  int data_;
};

class CpuTempMetric : public MetricQuerentInterface {
 public:
  CpuTempMetric() : data_(0) {}

  MetricQueryResult query(MetricDataInterface **m_data, std::string *err)
      override {
    data_ = 65;
    *m_data = new TemperatureMetricData(data_);
    return *m_data ? MetricQueryResult::OK : MetricQueryResult::ERROR;
  }

 private:
  int data_;
};

DCL_METRIC_QUERENT_CREATOR(PresenceMetric);
DCL_METRIC_QUERENT_CREATOR(CpuTempMetric);

DCL_METRIC_QUERENT_LIST({
  {"x-presence", "presence of X", _SYSMON_TYPE_TO_CREATOR_NAME(PresenceMetric)},
  {"cputemp", "CPU temperature", _SYSMON_TYPE_TO_CREATOR_NAME(CpuTempMetric)},
  {nullptr, nullptr}
});

}  // namespace sysmon
}  // namespace monitor
}  // namespace apollo
