
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
#ifndef CYBERTRON_BENCHMARK_COMPONENT_FOR_QA_WRITER_H
#define CYBERTRON_BENCHMARK_COMPONENT_FOR_QA_WRITER_H

#include "cybertron/cybertron.h"
#include "cybertron/component/component_base.h"
#include "cybertron/component/timer_component.h"
#include "cybertron/class_loader/class_loader.h"
#include "cybertron/proto/cybertron_whisper.pb.h"
#include "component_test.h"

using apollo::cybertron::TimerComponent;
using apollo::cybertron::Component;
using apollo::cybertron::Writer;
using apollo::cybertron::proto::CybertronWhisper;


namespace apollo {
namespace cybertron {

template <typename T>
class Writer;

}
}

class SenderComponent : public TimerComponent, public ComponentTest {
 public:
  SenderComponent();
  ~SenderComponent() {}
  bool Init() override;
  bool Proc() override;
};
CYBERTRON_REGISTER_COMPONENT(SenderComponent)


class OneChannelReceiverComponent : public Component<CybertronWhisper>, public ComponentTest {
 public:
  OneChannelReceiverComponent() {}
  bool Init() override;
  bool Proc(const std::shared_ptr<CybertronWhisper>& msg) override;
};
CYBERTRON_REGISTER_COMPONENT(OneChannelReceiverComponent)


class TwoChannelReceiverComponent : public Component<CybertronWhisper, CybertronWhisper>, public ComponentTest {
 public:
  TwoChannelReceiverComponent() {}
  bool Init() override;
  bool Proc(const WhisperCommon::ChannelPtr& whisper1,
            const WhisperCommon::ChannelPtr& whisper2) override;
};
CYBERTRON_REGISTER_COMPONENT(TwoChannelReceiverComponent)


class ThreeChannelReceiverComponent :
    public Component<CybertronWhisper, CybertronWhisper, CybertronWhisper>,
    public ComponentTest {
 public:
  ThreeChannelReceiverComponent() {}
  bool Init() override;
  bool Proc(const WhisperCommon::ChannelPtr& whisper1,
            const WhisperCommon::ChannelPtr& whisper2,
            const WhisperCommon::ChannelPtr& whisper3) override;
};
CYBERTRON_REGISTER_COMPONENT(ThreeChannelReceiverComponent)


#endif
