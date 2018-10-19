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

/**
 * @file
 */

#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/message_util.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"

namespace apollo {
namespace prediction {

/**
 * class FakePredictionComponent
 * This class generates fake prediction messages. The prediction message only
 * has valid headers.
 *
 * This tool is used to trigger modules that depends on prediction message.
 */

class FakePredictionComponent : public apollo::cyber::TimerComponent {
 public:
  bool Init() override {
    prediction_writer_ =
        node_->CreateWriter<PredictionObstacles>(FLAGS_prediction_topic);
    return true;
  }
  bool Proc() override {
    auto prediction = std::make_shared<PredictionObstacles>();
    common::util::FillHeader("fake_prediction", prediction.get());
    prediction_writer_->Write(prediction);
    return true;
  }

 private:
  std::shared_ptr<apollo::cyber::Writer<PredictionObstacles>>
      prediction_writer_;
};
CYBER_REGISTER_COMPONENT(FakePredictionComponent);

}  // namespace prediction
}  // namespace apollo
