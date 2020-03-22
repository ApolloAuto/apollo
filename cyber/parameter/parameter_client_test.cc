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

#include "cyber/parameter/parameter_client.h"

#include <memory>
#include <thread>
#include <vector>
#include "gtest/gtest.h"

#include "cyber/cyber.h"
#include "cyber/init.h"
#include "cyber/message/protobuf_factory.h"
#include "cyber/parameter/parameter_server.h"

namespace apollo {
namespace cyber {

class ParameterClientTest : public ::testing::Test {
 protected:
  ParameterClientTest() {
    apollo::cyber::Init("parameter_client_test");
    SetState(STATE_INITIALIZED);
    node_ = CreateNode("parameter_server");
  }

  virtual void SetUp() {
    // Called before every TEST_F(ParameterClientTest, *)
    ps_.reset(new ParameterServer(node_));
    pc_.reset(new ParameterClient(node_, "parameter_server"));
  }

  virtual void TearDown() {
    // Called after every TEST_F(ParameterClientTest, *)
    ps_.reset();
    pc_.reset();
  }

 protected:
  std::shared_ptr<Node> node_;
  std::unique_ptr<ParameterServer> ps_;
  std::unique_ptr<ParameterClient> pc_;
};

TEST_F(ParameterClientTest, set_parameter) {
  EXPECT_TRUE(pc_->SetParameter(Parameter("int", 1)));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  ps_.reset();
  EXPECT_FALSE(pc_->SetParameter(Parameter("int", 1)));
}

TEST_F(ParameterClientTest, get_parameter) {
  ps_->SetParameter(Parameter("int", 1));
  Parameter parameter;
  EXPECT_TRUE(pc_->GetParameter("int", &parameter));
  EXPECT_EQ("int", parameter.Name());
  EXPECT_EQ(1, parameter.AsInt64());
  EXPECT_FALSE(pc_->GetParameter("double", &parameter));

  ps_.reset();
  EXPECT_FALSE(pc_->GetParameter("int", &parameter));
}

TEST_F(ParameterClientTest, list_parameter) {
  ps_->SetParameter(Parameter("int", 1));
  std::vector<Parameter> parameters;
  EXPECT_TRUE(pc_->ListParameters(&parameters));
  EXPECT_EQ(1, parameters.size());
  EXPECT_EQ("int", parameters[0].Name());
  EXPECT_EQ(1, parameters[0].AsInt64());

  ps_.reset();
  EXPECT_FALSE(pc_->ListParameters(&parameters));
}

}  // namespace cyber
}  // namespace apollo
