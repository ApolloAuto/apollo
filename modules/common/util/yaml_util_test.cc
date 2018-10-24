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

#include "modules/common/util/yaml_util.h"

#include "gtest/gtest.h"

#include "modules/common/util/file.h"
#include "modules/common/util/testdata/simple.pb.h"
#include "modules/common/util/util.h"

namespace apollo {
namespace common {
namespace util {

TEST(YamlUtilTest, YamlToProto) {
  const std::string kInputYamlFile =
      "/apollo/modules/common/util/testdata/simple_proto.yaml";
  const std::string kExpectedProtoFile =
      "/apollo/modules/common/util/testdata/simple_proto.pb.txt";

  test::SimpleRepeatedMessage loaded_proto;
  EXPECT_TRUE(YamlToProto(kInputYamlFile, &loaded_proto));

  test::SimpleRepeatedMessage expected_proto;
  ASSERT_TRUE(GetProtoFromASCIIFile(kExpectedProtoFile, &expected_proto));

  EXPECT_TRUE(IsProtoEqual(expected_proto, loaded_proto));
}

}  // namespace util
}  // namespace common
}  // namespace apollo
