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

#include "cyber/transport/dispatcher/dispatcher.h"

#include <memory>
#include <vector>
#include "gtest/gtest.h"

#include "cyber/common/util.h"
#include "cyber/proto/unit_test.pb.h"
#include "cyber/transport/common/identity.h"

namespace apollo {
namespace cyber {
namespace transport {

class DispatcherTest : public ::testing::Test {
 protected:
  DispatcherTest() : attr_num_(100) {
    for (int i = 0; i < attr_num_; ++i) {
      auto channel_name = "channel_" + std::to_string(i);
      RoleAttributes attr;
      attr.set_channel_name(channel_name);
      attr.set_channel_id(common::Hash(channel_name));
      Identity self_id;
      attr.set_id(self_id.HashValue());
      self_attrs_.emplace_back(attr);

      Identity oppo_id;
      attr.set_id(oppo_id.HashValue());
      oppo_attrs_.emplace_back(attr);
    }
  }

  virtual ~DispatcherTest() {
    self_attrs_.clear();
    oppo_attrs_.clear();
    dispatcher_.Shutdown();
  }

  virtual void SetUp() {
    for (int i = 0; i < attr_num_; ++i) {
      dispatcher_.AddListener<proto::Chatter>(
          self_attrs_[i],
          [](const std::shared_ptr<proto::Chatter>&, const MessageInfo&) {});

      dispatcher_.AddListener<proto::Chatter>(
          self_attrs_[i], oppo_attrs_[i],
          [](const std::shared_ptr<proto::Chatter>&, const MessageInfo&) {});
    }
  }

  virtual void TearDown() {
    for (int i = 0; i < attr_num_; ++i) {
      dispatcher_.RemoveListener<proto::Chatter>(self_attrs_[i]);
      dispatcher_.RemoveListener<proto::Chatter>(self_attrs_[i],
                                                 oppo_attrs_[i]);
    }
  }

  int attr_num_;
  Dispatcher dispatcher_;
  std::vector<RoleAttributes> self_attrs_;
  std::vector<RoleAttributes> oppo_attrs_;
};

TEST_F(DispatcherTest, shutdown) { dispatcher_.Shutdown(); }

TEST_F(DispatcherTest, add_and_remove_listener) {
  RoleAttributes self_attr;
  self_attr.set_channel_name("add_listener");
  self_attr.set_channel_id(common::Hash("add_listener"));
  Identity self_id;
  self_attr.set_id(self_id.HashValue());

  RoleAttributes oppo_attr;
  oppo_attr.set_channel_name("add_listener");
  oppo_attr.set_channel_id(common::Hash("add_listener"));
  Identity oppo_id;
  oppo_attr.set_id(oppo_id.HashValue());

  dispatcher_.RemoveListener<proto::Chatter>(self_attr);
  dispatcher_.RemoveListener<proto::Chatter>(self_attr, oppo_attr);

  dispatcher_.AddListener<proto::Chatter>(
      self_attr, [](const std::shared_ptr<proto::Chatter>&,
                    const MessageInfo&) { AINFO << "I'm listener a."; });
  EXPECT_TRUE(dispatcher_.HasChannel(common::Hash("add_listener")));

  dispatcher_.AddListener<proto::Chatter>(
      self_attr, [](const std::shared_ptr<proto::Chatter>&,
                    const MessageInfo&) { AINFO << "I'm listener b."; });
  dispatcher_.RemoveListener<proto::Chatter>(self_attr);
  dispatcher_.Shutdown();

  dispatcher_.AddListener<proto::Chatter>(
      self_attr, oppo_attr,
      [](const std::shared_ptr<proto::Chatter>&, const MessageInfo&) {
        AINFO << "I'm listener c.";
      });

  dispatcher_.AddListener<proto::Chatter>(
      self_attr, oppo_attr,
      [](const std::shared_ptr<proto::Chatter>&, const MessageInfo&) {
        AINFO << "I'm listener d.";
      });

  dispatcher_.RemoveListener<proto::Chatter>(self_attr, oppo_attr);
}

TEST_F(DispatcherTest, has_channel) {
  for (int i = 0; i < attr_num_; ++i) {
    auto channel_name = "channel_" + std::to_string(i);
    EXPECT_TRUE(dispatcher_.HasChannel(common::Hash(channel_name)));
  }
  EXPECT_FALSE(dispatcher_.HasChannel(common::Hash("has_channel")));
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo
