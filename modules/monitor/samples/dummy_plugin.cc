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

#include "dummy_plugin.h"

namespace apollo {
namespace monitor {
namespace sysmon {

DCL_SYSMON_PLUGIN_API_VER(PLUGIN_API_VERSION_1);

DCL_METRIC_QUERENT_CREATOR(DummyMetric);

DCL_METRIC_QUERENT_LIST({
  {"dummy", "dummy metric querent", _SYSMON_TYPE_TO_CREATOR_NAME(DummyMetric)},
  {nullptr, nullptr, nullptr}
});

typedef MetricDataSingleDes<int> SingleIntDes;

DCL_METRIC_DATADES_MAKER(SingleIntDes);

DCL_METRIC_DATADES_LIST({
  {"singleint", "dummy metric data deserilizer",
      _SYSMON_TYPE_TO_CREATOR_NAME(SingleIntDes)},
  {nullptr, nullptr, nullptr}
});

}}}
