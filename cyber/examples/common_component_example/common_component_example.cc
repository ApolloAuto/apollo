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
#include "cyber/examples/common_component_example/common_component_example.h"

bool CommonComponentSample::Init() {
  AINFO << "Commontest component init";
  return true;
}

bool CommonComponentSample::Proc(const std::shared_ptr<Driver>& msg0,
                                 const std::shared_ptr<Driver>& msg1) {
  AINFO << "Start common component Proc [" << msg0->msg_id() << "] ["
        << msg1->msg_id() << "]";
  return true;
}
