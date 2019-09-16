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

#include "cyber/transport/shm/notifier_factory.h"

#include <string>

#include "cyber/common/global_data.h"
#include "cyber/common/log.h"
#include "cyber/transport/shm/condition_notifier.h"
#include "cyber/transport/shm/multicast_notifier.h"

namespace apollo {
namespace cyber {
namespace transport {

using common::GlobalData;

auto NotifierFactory::CreateNotifier() -> NotifierPtr {
  std::string notifier_type(ConditionNotifier::Type());
  auto& g_conf = GlobalData::Instance()->Config();
  if (g_conf.has_transport_conf() && g_conf.transport_conf().has_shm_conf() &&
      g_conf.transport_conf().shm_conf().has_notifier_type()) {
    notifier_type = g_conf.transport_conf().shm_conf().notifier_type();
  }

  ADEBUG << "notifier type: " << notifier_type;

  if (notifier_type == MulticastNotifier::Type()) {
    return CreateMulticastNotifier();
  } else if (notifier_type == ConditionNotifier::Type()) {
    return CreateConditionNotifier();
  }

  AINFO << "unknown notifier, we use default notifier: " << notifier_type;
  return CreateConditionNotifier();
}

auto NotifierFactory::CreateConditionNotifier() -> NotifierPtr {
  return ConditionNotifier::Instance();
}

auto NotifierFactory::CreateMulticastNotifier() -> NotifierPtr {
  return MulticastNotifier::Instance();
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo
