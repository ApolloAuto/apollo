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

#include "cyber/tools/cyber_monitor/general_message.h"

#include <iomanip>
#include <numeric>
#include <sstream>
#include <vector>

#include "cyber/tools/cyber_monitor/general_channel_message.h"
#include "cyber/tools/cyber_monitor/screen.h"

namespace {

/**
 * if map has string keys, lexically sort them
 */
std::vector<int> SortProtobufMapByKeys(
    const google::protobuf::Message& message,
    const google::protobuf::FieldDescriptor* field,
    const google::protobuf::Reflection& reflection, const int size) {
  std::vector<int> output;
  if (0 == size) {
    return output;
  }
  if (field->cpp_type() == google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE) {
    const ::google::protobuf::Message& item =
        reflection.GetRepeatedMessage(message, field, 0);
    const ::google::protobuf::FieldDescriptor* item_fd =
        item.GetDescriptor()->FindFieldByName("key");
    if (item_fd && field->is_map() &&
        ::google::protobuf::FieldDescriptor::Type::TYPE_STRING ==
            item_fd->type()) {
      std::vector<std::pair<std::string, int>> key_indices;
      key_indices.reserve(size);
      for (int i = 0; i < size; ++i) {
        const ::google::protobuf::Message& item =
            reflection.GetRepeatedMessage(message, field, i);
        const ::google::protobuf::FieldDescriptor* item_fd =
            item.GetDescriptor()->FindFieldByName("key");
        const std::string key(item.GetReflection()->GetString(item, item_fd));
        key_indices.emplace_back(key, i);
      }
      std::sort(key_indices.begin(), key_indices.end());
      output.reserve(size);
      for (const std::pair<std::string, int>& key_index : key_indices) {
        output.push_back(key_index.second);
      }
    }
  }

  if (output.empty()) {
    output.resize(size);
    std::iota(output.begin(), output.end(), 0);
  }
  return output;
}
}  // namespace

GeneralMessage::GeneralMessage(GeneralMessageBase* parent,
                               const google::protobuf::Message* msg,
                               const google::protobuf::Reflection* reflection,
                               const google::protobuf::FieldDescriptor* field)
    : GeneralMessageBase(parent),
      item_index_(0),
      is_folded_(true),
      field_(field),
      message_ptr_(msg),
      reflection_ptr_(reflection) {}

int GeneralMessage::Render(const Screen* s, int key) {
  s->SetCurrentColor(Screen::WHITE_BLACK);
  int line_no = 0;
  {
    RenderableMessage* p = this;
    while (p->parent()->parent()->parent()) {
      p = p->parent();
    }

    GeneralChannelMessage* channel_msg_ptr =
        static_cast<GeneralChannelMessage*>(p->parent());
    s->AddStr(0, line_no++, "ChannelName: ");
    s->AddStr(channel_msg_ptr->GetChannelName().c_str());

    s->AddStr(0, line_no++, "MessageType: ");
    s->AddStr(channel_msg_ptr->message_type().c_str());

    std::ostringstream out_str;
    out_str << std::fixed << std::setprecision(FrameRatio_Precision)
            << channel_msg_ptr->frame_ratio();
    s->AddStr(0, line_no++, "FrameRatio: ");
    s->AddStr(out_str.str().c_str());

    clear();

    auto channel_msg = channel_msg_ptr->CopyMsgPtr();
    if (!channel_msg_ptr->raw_msg_class_->ParseFromString(
            channel_msg->message)) {
      s->AddStr(0, line_no++,
                "Cannot Parse the message for Real-Time Updating");
      return line_no;
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

      if (size <= item_index_) {
        out_str.str("");
        out_str << "The item [" << item_index_ << "] has been empty !!!";
        s->AddStr(0, line_no++, out_str.str().c_str());
        return line_no;
      }

      if (key == ',') {
        is_folded_ = !is_folded_;
      } else if (is_folded_) {
        switch (key) {
          case 'n':
          case 'N':
            ++item_index_;
            if (item_index_ >= size) {
              item_index_ = 0;
            }
            break;

          case 'm':
          case 'M':
            --item_index_;
            if (item_index_ < 0) {
              item_index_ = size - 1;
            }
            break;

          default: {
          }
        }
      }

      int lcount = LineCountOfField(*message_ptr_, s->Width(), field_,
                                    reflection_ptr_, is_folded_);
      page_item_count_ = s->Height() - line_no - 8;
      if (page_item_count_ < 1) {
        page_item_count_ = 1;
      }
      pages_ = lcount / page_item_count_ + 1;
      SplitPages(key);
      int jump_lines = page_index_ * page_item_count_;
      const std::vector<int> indices(
          SortProtobufMapByKeys(*message_ptr_, field_, *reflection_ptr_, size));
      if (is_folded_) {
        GeneralMessageBase::PrintField(this, *message_ptr_, &jump_lines, s,
                                       &line_no, 0, reflection_ptr_, field_,
                                       indices[item_index_]);
      } else {
        for (const int index : indices) {
          GeneralMessageBase::PrintField(this, *message_ptr_, &jump_lines, s,
                                         &line_no, 0, reflection_ptr_, field_,
                                         index);
        }
      }
    }
  }

  s->ClearCurrentColor();
  return line_no;
}
