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

#ifndef MODULES_TOOLS_NAVI_GENERATOR_BACKEND_NAVI_GENERATOR_H_
#define MODULES_TOOLS_NAVI_GENERATOR_BACKEND_NAVI_GENERATOR_H_

#include <memory>
#include <string>

#include "CivetServer.h"

#include "modules/common/apollo_app.h"
#include "modules/tools/navi_generator/backend/hmi/topics_updater.h"
#include "modules/tools/navi_generator/backend/webserver/navi_generator_websocket.h"

namespace apollo {
namespace navi_generator {

class NaviGenerator : public apollo::common::ApolloApp {
 public:
  std::string Name() const override;

  apollo::common::Status Init() override;

  apollo::common::Status Start() override;

  void Stop() override;

  virtual ~NaviGenerator() = default;

 private:
  void TerminateProfilingMode(const ros::TimerEvent &event);

  void CheckAdapters();

  ros::Timer exit_timer_;

  std::unique_ptr<TopicsUpdater> topics_updater_;
  std::unique_ptr<CivetServer> server_;
  std::unique_ptr<NaviGeneratorWebSocket> websocket_;
};

}  // namespace navi_generator
}  // namespace apollo

#endif  // MODULES_TOOLS_NAVI_GENERATOR_BACKEND_REF_LINE_PARTNER_H_
