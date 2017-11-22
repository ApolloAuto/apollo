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
#ifndef MODULES_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_OUTPUT_SUBNODE_H
#define MODULES_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_OUTPUT_SUBNODE_H

#include <map>
#include <string>
#include <array>

#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>

#include "modules/perception/onboard/subnode.h"
#include "modules/perception/traffic_light/base/image_lights.h"

namespace apollo {
namespace perception {
namespace traffic_light {

class TLProcData;

class TLOutputSubnode : public Subnode {
 public:
  TLOutputSubnode() = default;
  virtual ~TLOutputSubnode() {
    _proc_data = nullptr;
  }

  virtual StatusCode proc_events() override;

 protected:
  virtual bool init_internal() override;
 private:
  bool init_shared_data();
  bool init_output_stream();

  bool transform_message(const Event &event,
                         const std::shared_ptr<ImageLights> &lights,
                         boost::shared_ptr<std_msgs::String> *msg) const;
  bool proc_upstream_data(const Event &event);

 private:
  std::unique_ptr<StreamOutput> _output_stream;
  TLProcData *_proc_data = nullptr;

  // save BGR colors
  //static std::map<std::string, std::array<int, 3> > _s_color_table;

  DISALLOW_COPY_AND_ASSIGN(TLOutputSubnode);
};

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_OUTPUT_SUBNODE_H
