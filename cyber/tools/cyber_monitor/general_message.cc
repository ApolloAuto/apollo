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
      is_folded_(true),
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

    GeneralChannelMessage* channelMsgPtr =
        static_cast<GeneralChannelMessage*>(p->parent());
    s->AddStr(0, lineNo++, "ChannelName: ");
    s->AddStr(channelMsgPtr->GetChannelName().c_str());

    s->AddStr(0, lineNo++, "MessageType: ");
    s->AddStr(channelMsgPtr->message_type().c_str());

    std::ostringstream outStr;
    outStr << std::fixed << std::setprecision(FrameRatio_Precision)
           << channelMsgPtr->frame_ratio();
    s->AddStr(0, lineNo++, "FrameRatio: ");
    s->AddStr(outStr.str().c_str());

    clear();

    auto channelMsg = channelMsgPtr->CopyMsgPtr();
    if (!channelMsgPtr->raw_msg_class_->ParseFromString(channelMsg->message)) {
      s->AddStr(0, lineNo++, "Cannot Parse the message for Real-Time Updating");
      return;
    }

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

      if (size <= itemIndex_) {
        outStr.str("");
        outStr << "The item [" << itemIndex_ << "] has been empty !!!";
        s->AddStr(0, lineNo++, outStr.str().c_str());
        return;
      }

      if (key == ',') {
        is_folded_ = !is_folded_;
      } else if (is_folded_) {
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
      }

      int lcount = lineCountOfField(*message_ptr_, s->Width(), field_,
                                    reflection_ptr_, is_folded_);
      page_item_count_ = s->Height() - lineNo - 8;
      if (page_item_count_ < 1) page_item_count_ = 1;
      pages_ = lcount / page_item_count_ + 1;
      SplitPages(key);
      int jumpLines = page_index_ * page_item_count_;
      if (is_folded_) {
        GeneralMessageBase::PrintField(this, *message_ptr_, jumpLines, s,
                                       lineNo, 0, reflection_ptr_, field_,
                                       itemIndex_);
      } else {
        for (int i = 0; i < size; ++i) {
          GeneralMessageBase::PrintField(this, *message_ptr_, jumpLines, s,
                                         lineNo, 0, reflection_ptr_, field_, i);
        }
      }
    }
  }

  s->ClearCurrentColor();
}
