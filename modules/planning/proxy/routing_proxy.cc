/***************************************************************************
 *
 * Copyright (c) 2017 Baidu.com, Inc. All Rights Reserved
 *
 **************************************************************************/

/**
 * @file routing_proxy.cpp
 **/

#include "modules/planning/proxy/routing_proxy.h"
#include "modules/common/util/file.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::ErrorCode;

apollo::common::Status RoutingProxy::Init() {
  // FIXME(all): change to use real routing server
  std::string routing_file("modules/map/data/garage.pb.txt");

  if (!::apollo::common::util::GetProtoFromFile(routing_file, &routing_)) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "Unable to load routing file: " + routing_file);
  }
  return Status::OK();
}

void RoutingProxy::set_routing(
    const ::apollo::hdmap::RoutingResult &routing_result) {
  routing_.CopyFrom(routing_result);
}

const ::apollo::hdmap::RoutingResult &RoutingProxy::routing() const {
  return routing_;
}

}  // namespace planning
}  // namespace apollo
