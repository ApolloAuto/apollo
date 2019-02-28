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

/**
 * @file os_interface.h
 * @brief define v2x proxy module and apollo os interface
 */

#pragma once

#include <memory>

#include "cyber/cyber.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/v2x/common/v2x_proxy_gflags.h"
#include "modules/v2x/proto/v2x_traffic_light.pb.h"

namespace apollo {
namespace v2x {

class OsInterFace {
 public:
  OsInterFace();

  ~OsInterFace();

  inline void PrintModuleDetails() {
    AINFO << "[module name: ]"
          << "v2x_os_interface"
          << " "
          << "[localization topic name: ]"
          << localization_reader_->GetChannelName() << " "
          << "[perception topic name: ]"
          << perception_obstacle_reader_->GetChannelName() << " "
          << "[v2x obstacles topic name: ]"
          << v2x_obstacle_writer_->GetChannelName() << " "
          << "[v2x trafficlight topic name: ]"
          << v2x_trafficlight_writer_->GetChannelName();
  }

  bool InitFlag() { return init_flag_; }
  /*function that get localization msg from apollo os
  @param output localization type msg shared ptr
  */
  void GetLocalizationFromOs(
      const std::shared_ptr<apollo::localization::LocalizationEstimate> &msg);

  /*function that get perception msg from apollo os
  @param output perception obstacles type msg shared ptr
  */
  void GetObstaclesFromOs(
      const std::shared_ptr<apollo::perception::PerceptionObstacles> &msg);

  /*function that send trafficlight msg to apollo os
  @param input trafficlight type msg shared ptr
  */
  void SendV2xTrafficLightToOs(
      const std::shared_ptr<IntersectionTrafficLightData> &msg);

  /*function that send obstacles msg to apollo os
  @param input obstacles type msg shared ptr
  */
  void SendV2xObstaclesToOs(
      const std::shared_ptr<apollo::perception::PerceptionObstacles> &msg);

 private:
  /*template function that get msg from apollo os according to type
  @param input  cybertron reader
  @param output template type msg shared ptr
  */
  template <typename MessageT>
  void GetMsgFromOs(const cyber::Reader<MessageT> *reader,
                    const std::shared_ptr<MessageT> &msg) {
    node_->Observe();
    if (reader->Empty()) {
      AINFO_EVERY(100) << "Has not received any data from "
                       << reader->GetChannelName();
      return;
    }
    msg->CopyFrom(*(reader->GetLatestObserved()));
  }

  /*template function that send msg to apollo os according to type
  @param input  cybertron writer
  @param input  template type msg shared ptr
  */
  template <typename MessageT>
  void SendMsgToOs(cyber::Writer<MessageT> *writer,
                   const std::shared_ptr<MessageT> &msg) {
    if (writer == nullptr) {
      AERROR << "Writer in not valid";
      return;
    }
    if (writer->Write(msg) == true) {
      ADEBUG << "Write msg success to: " << writer->GetChannelName();
    } else {
      AERROR << "Write msg failed to: " << writer->GetChannelName();
    }
  }
  bool InitReaders();
  bool InitWriters();
  std::unique_ptr<cyber::Node> node_;
  std::shared_ptr<cyber::Reader<apollo::localization::LocalizationEstimate>>
      localization_reader_ = nullptr;
  std::shared_ptr<cyber::Reader<apollo::perception::PerceptionObstacles>>
      perception_obstacle_reader_ = nullptr;
  std::shared_ptr<cyber::Writer<apollo::perception::PerceptionObstacles>>
      v2x_obstacle_writer_ = nullptr;
  std::shared_ptr<cyber::Writer<IntersectionTrafficLightData>>
      v2x_trafficlight_writer_ = nullptr;
  bool init_flag_ = false;
};
}  // namespace v2x
}  // namespace apollo
