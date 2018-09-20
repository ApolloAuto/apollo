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

#ifndef TOOLS_CVT_MONITOR_GENERAL_MESSAGE_BASE_H_
#define TOOLS_CVT_MONITOR_GENERAL_MESSAGE_BASE_H_

#include "cybertron/cybertron.h"
#include "renderable_message.h"

class RepeatedItemsMessage;
class Screen;

class GeneralMessageBase : public RenderableMessage {
 public:
  virtual int type(void) const = 0;  // all children's type should return one
                                     // identical value, e.g. { return Type;}

 protected:
  enum { Type = 0 };
  static void PrintMessage(GeneralMessageBase* baseMsg,
                           const google::protobuf::Message& msg,
                           const Screen* s, unsigned& lineNo, int indent,
                           int jumpLines = 0);

  static int lineCount(const google::protobuf::Message& msg, int screenWidth);

  void insertRepeatedMessage(int lineNo, GeneralMessageBase* item){
      children_map_.insert(std::make_pair(lineNo, item));
  }

  RenderableMessage* Child(int lineNo) const;

  explicit GeneralMessageBase(RenderableMessage* parent = nullptr) : RenderableMessage(parent), children_map_() {}
  ~GeneralMessageBase(void) {
    clear();
  }

  void clear(void){
    for (auto& iter : children_map_) {
      delete iter.second;
    }

    children_map_.clear();
  }

  GeneralMessageBase(const GeneralMessageBase&) = delete;
  GeneralMessageBase& operator=(const GeneralMessageBase&) = delete;

  std::map<const int /* lineNo */, GeneralMessageBase*> children_map_;
};

#endif  // TOOLS_CVT_MONITOR_GENERAL_MESSAGE_BASE_H_
