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

#include "modules/localization/msf/local_tool/data_extraction/rosbag_reader.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

namespace apollo {
namespace localization {
namespace msf {

RosbagReader::RosbagReader() {}

RosbagReader::~RosbagReader() {}

void RosbagReader::Subscribe(const std::string &topic,
                             BaseExporter::OnRosmsgCallback call_back,
                             BaseExporter::Ptr exporter) {
  call_back_map_[topic] = std::make_pair(exporter, call_back);
  topics_.push_back(topic);
}

void RosbagReader::Read(const std::string &file_name) {
  rosbag::Bag bag;
  bag.open(file_name, rosbag::bagmode::Read);

  rosbag::View view(bag, rosbag::TopicQuery(topics_));

  foreach (rosbag::MessageInstance const m, view) {
    const std::string tp = m.getTopic();
    std::cout << "Read topic: " << tp << std::endl;

    std::unordered_map<std::string,
                       std::pair<BaseExporter::Ptr,
                                 BaseExporter::OnRosmsgCallback>>::iterator it =
        call_back_map_.find(tp);
    if (it != call_back_map_.end()) {
      BaseExporter &exporter = *(it->second.first);
      BaseExporter::OnRosmsgCallback call_back = it->second.second;

      (exporter.*call_back)(m);
    }
  }

  bag.close();
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
