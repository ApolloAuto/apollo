/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "cyber/ros_bridge/converters/examples/ros_apollo_converter/ros_apollo_converter.h"

namespace apollo {
namespace cyber {

bool RosApolloConverter::ConvertMsg(InputTypes<InputMsgPtr>& in,
                                    OutputTypes<OutputMsgPtr>& out) {
  AERROR << "123";
#ifdef ENABLE_ROS_MSG
  auto in_msg = std::get<0>(in.values);
  auto out_msg = std::get<0>(out.values);
  out_msg->set_text(in_msg->data.c_str());
#endif
  return true;
}

}  // namespace cyber
}  // namespace apollo
