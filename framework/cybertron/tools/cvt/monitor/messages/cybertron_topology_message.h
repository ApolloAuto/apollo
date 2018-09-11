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

#ifndef CYBERTRONTOPOLOGYMESSAGE_H
#define CYBERTRONTOPOLOGYMESSAGE_H

#include <map>
#include "renderable_message.h"

namespace apollo {
namespace cybertron {
namespace proto {
class ChangeMsg;
}  // proto
}  // cybertron
}  // apollo

class ChannelMessage;

class CybertronTopologyMessage : public RenderableMessage {
 public:
  explicit CybertronTopologyMessage();
  ~CybertronTopologyMessage();

  void Render(const Screen* s, int key) override;
  RenderableMessage* Child(int index) const override;

  void TopologyChanged(const apollo::cybertron::proto::ChangeMsg& change_msg);

 private:
  void ChangeState(int key);

  enum class SecondColumnType { MessageType, MessageFrameRatio };
  SecondColumnType second_column_;

  int pages_;
  int page_item_count_;
  int page_index_;
  int col1_width_;

  std::map<std::string, ChannelMessage*> all_channels_map_;
};

#endif  // CYBERTRONTOPOLOGYMESSAGE_H
