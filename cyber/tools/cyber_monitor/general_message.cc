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

#include "./general_message.h"
#include "./general_channel_message.h"
#include "./screen.h"

#include <iomanip>
#include <sstream>

GeneralMessage::GeneralMessage(GeneralMessageBase* parent,
                               const google::protobuf::Message* msg,
                               const google::protobuf::Reflection* reflection,
                               const google::protobuf::FieldDescriptor* field)
    : GeneralMessageBase(parent),
      itemIndex_(0),
      field_(field),
      message_ptr_(msg),
      reflection_ptr_(reflection) {}

void GeneralMessage::Render(const Screen* s, int key) {
  s->SetCurrentColor(Screen::WHITE_BLACK);

  {
    unsigned lineNo = 0;

    RenderableMessage* p = this;
    while (p->parent()->parent()->parent()) {
      p = p->parent();
    }

    GeneralChannelMessage* parentPtr =
        static_cast<GeneralChannelMessage*>(p->parent());
    s->AddStr(0, lineNo++, "ChannelName: ");
    s->AddStr(parentPtr->GetChannelName().c_str());

    s->AddStr(0, lineNo++, "MessageType: ");
    s->AddStr(parentPtr->message_type().c_str());

    std::ostringstream outStr;
    outStr << std::fixed << std::setprecision(FrameRatio_Precision) << parentPtr->frame_ratio();
    s->AddStr(0, lineNo++, "FrameRatio: ");
    s->AddStr(outStr.str().c_str());

    clear();

    if (message_ptr_ && reflection_ptr_) {
      int size = 0;
      if (field_->is_repeated()) {
        size = reflection_ptr_->FieldSize(*message_ptr_, field_);
      } else {
        if (reflection_ptr_->HasField(*message_ptr_, field_) ||
            field_->containing_type()->options().map_entry()) {
          size = 1;
        }
      }

      switch (key) {
        case 'n':
        case 'N':
          ++itemIndex_;
          if (itemIndex_ >= size) itemIndex_ = 0;
          break;

        case 'm':
        case 'M':
          --itemIndex_;
          if (itemIndex_ < 0) itemIndex_ = size - 1;
          break;

        default: {}
      }

      int lcount =
          lineCountOfField(*message_ptr_, s->Width(), field_, reflection_ptr_);
      page_item_count_ = s->Height() - lineNo;
      pages_ = lcount / page_item_count_ + 1;
      SplitPages(key);

      int jumpLines = page_index_ * page_item_count_;

      jumpLines <<= 2;
      jumpLines /= 5;
      GeneralMessageBase::PrintField(this, *message_ptr_, jumpLines, s, lineNo,
                                     0, reflection_ptr_, field_, itemIndex_);
    }
  }

  s->ClearCurrentColor();
}
