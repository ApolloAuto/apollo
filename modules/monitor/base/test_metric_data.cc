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

#include <iostream>

#include "modules/monitor/sysmon/base/metric_data_single.h"

#include "gtest/gtest.h"

namespace apollo {
namespace monitor {
namespace sysmon {

TEST(MetricDataSingle, Serilizer) {
  int val = 280;
  MetricDataSingle<int> md(val);
  EXPECT_EQ(val, md.get());

  int v2 = 101;
  md.set(v2);
  EXPECT_EQ(v2, md.get());
  EXPECT_EQ("101", md.to_json_str());
}

TEST(MetricDataSingleDes, Deserilizer) {
  MetricDataSingleDes<int> md;
  std::string empty("");
  EXPECT_EQ(false, md.rcv_json_str(empty));
  std::string s280("280");
  EXPECT_EQ(true, md.rcv_json_str(s280));
  EXPECT_EQ(280, md.get());
}

TEST(MetricDataSingleSpecialized, smetric) {
  FanSpeedMetricData fs(1050);
  EXPECT_EQ(1050, fs.get());

  PresenceMetricDataDes pd;
  EXPECT_EQ(true, pd.rcv_json_str("1"));
  EXPECT_EQ("X", pd.to_str_simp());
  EXPECT_EQ(true, pd.rcv_json_str("0"));
  EXPECT_EQ("-", pd.to_str_simp());

  nlohmann::json jcfg;
  jcfg["missing"] = "not there";
  pd.config(&jcfg);
  EXPECT_EQ("not there", pd.to_str_verbose());

  FanSpeedMetricDataDes fs_d;
  EXPECT_EQ(true, fs_d.rcv_json_str("1050"));
  EXPECT_EQ("1050rpm", fs_d.to_str_verbose());

  TemperatureMetricDataDes td(TemperatureMetricDataDes::Unit::TEMP_C);
  EXPECT_EQ(true, td.rcv_json_str("75"));
  EXPECT_EQ("75C", td.to_str_verbose());
}

}  // namespace sysmon
}  // namespace monitor
}  // namespace apollo
