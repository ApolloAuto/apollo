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

#include <gtest/gtest.h>
#include <memory>
#include <vector>

#include "cyber/cyber.h"
#include "cyber/init.h"
#include "cyber/message/protobuf_factory.h"
#include "cyber/parameter/parameter_server.h"

namespace apollo {
namespace cyber {

using apollo::cyber::proto::Param;
using apollo::cyber::proto::ParamType;

class ParameterServerTest : public ::testing::Test {
 protected:
  ParameterServerTest() {}
  virtual ~ParameterServerTest() {}

  std::shared_ptr<Node> node_;
  std::shared_ptr<ParameterServer> ps_;

  virtual void SetUp() {
    // Called before every TEST_F(ParameterServerTest, *)
    apollo::cyber::Init("parameter_server_test");
    node_ = CreateNode("parameter_server");
    ps_.reset(new ParameterServer(node_));
  }

  virtual void TearDown() {
    // Called after every TEST_F(ParameterServerTest, *)
  }
};

TEST_F(ParameterServerTest, set_parameter) {
  ps_->SetParameter(Parameter("int", 1));
}

TEST_F(ParameterServerTest, get_parameter) {
  ps_->SetParameter(Parameter("int", 1));
  Parameter parameter;
  ps_->GetParameter("int", &parameter);
  EXPECT_EQ("int", parameter.Name());
  EXPECT_EQ(1, parameter.AsInt64());
}

TEST_F(ParameterServerTest, list_parameter) {
  ps_->SetParameter(Parameter("int", 1));
  std::vector<Parameter> parameters;
  ps_->ListParameters(&parameters);
  EXPECT_EQ(1, parameters.size());
  EXPECT_EQ("int", parameters[0].Name());
  EXPECT_EQ(1, parameters[0].AsInt64());
}

}  // namespace cyber
}  // namespace apollo
