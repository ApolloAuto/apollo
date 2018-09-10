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

#include "cybertron/cybertron.h"
#include "cybertron/proto/chatter.pb.h"
#include "cybertron/message/py_message.h"
#include "python/wrapper/py_node.h"

using apollo::cybertron::message::PyMessageWrap;

apollo::cybertron::PyReader *pr = NULL;

int cbfun(const char *channel_name) {
  AINFO << "recv->[ " << channel_name << " ]";
  if (pr)
    AINFO << "read->[ " << pr->read() << " ]";
}

int main(int argc, char *argv[]) {
  apollo::cybertron::proto::Chatter chat;    
  apollo::cybertron::PyNode node("listener");
  pr = node.create_reader("channel/chatter", chat.GetTypeName());
  pr->register_func(cbfun);
  
  apollo::cybertron::WaitForShutdown();
  delete pr;

  return 0;
}
