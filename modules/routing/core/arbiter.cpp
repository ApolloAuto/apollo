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

#include "core/arbiter.h"

#include "core/navigator.h"
#include "std_msgs/String.h"

namespace adu {
namespace routing {

Arbiter::Arbiter() {
    _node_handle_ptr.reset(new ros::NodeHandle(FLAGS_node_namespace));
    if (FLAGS_enable_old_routing) {
        ROS_INFO("Use old routing request and result!");
        _service = _node_handle_ptr->advertiseService(FLAGS_signal_probe_service,
                                                      &Arbiter::on_request_old_routing,
                                                      this);
    } else {
        ROS_INFO("Use new routing request and result!");
        _service = _node_handle_ptr->advertiseService(FLAGS_signal_probe_service,
                                                      &Arbiter::on_request,
                                                      this);
    }
    _publisher = _node_handle_ptr->advertise<std_msgs::String>(FLAGS_route_topic_for_broadcast, 1);
    std::string graph_path = FLAGS_graph_dir + "/" + FLAGS_graph_file_name;
    ROS_INFO("Use routing topology graph path: %s", graph_path.c_str());
    _navigator_ptr.reset(new Navigator(graph_path));

    // set init ok for ADS test, when it knows routing is ok, it can replay bags
    _node_handle_ptr->setParam(FLAGS_rosparam_name_routing_init_status, true);
}

Arbiter::~Arbiter() { }

bool Arbiter::on_request(arbiter::routing_signal::Request& req,
                         arbiter::routing_signal::Response& res) {
    ROS_INFO("Get new routing request!!!");
    ::adu::common::router::RoutingRequest request_proto;
    ::adu::common::router::RoutingResult response_proto;
    if (!request_proto.ParseFromString(req.routing_request.data)) {
        ROS_ERROR("The request proto is invalid.");
        return false;
    }
    if (!_navigator_ptr->search_route(request_proto, &response_proto)) {
        ROS_ERROR("Failed to search route with navigator.");
        return false;
    }
    if (!response_proto.SerializeToString(&(res.routing_response.data))) {
        ROS_ERROR("Failed to serialize routing response.");
        return false;
    }
    if (request_proto.broadcast()) {
        std_msgs::String publish_msg;
        if (!response_proto.SerializeToString(&(publish_msg.data))) {
            ROS_ERROR("Failed to serialize routing response.");
            return false;
        }
        _publisher.publish(publish_msg);
    }
    return true;
}

bool Arbiter::on_request_old_routing(arbiter::routing_signal::Request& req,
                                     arbiter::routing_signal::Response& res) {
    ROS_INFO("Get old routing request!!!");
    ::adu::common::routing::RoutingRequest request_proto;
    ::adu::common::routing::RoutingResult response_proto;
    if (!request_proto.ParseFromString(req.routing_request.data)) {
        ROS_ERROR("The request proto is invalid.");
        return false;
    }
    if (!_navigator_ptr->search_route(request_proto, &response_proto)) {
        ROS_ERROR("Failed to search route with navigator.");
        return false;
    }
    if (!response_proto.SerializeToString(&(res.routing_response.data))) {
        ROS_ERROR("Failed to serialize routing response.");
        return false;
    }
    if (request_proto.broadcast()) {
        std_msgs::String publish_msg;
        if (!response_proto.SerializeToString(&(publish_msg.data))) {
            ROS_ERROR("Failed to serialize routing response.");
            return false;
        }
        _publisher.publish(publish_msg);
    }
    return true;
}

bool Arbiter::run() {
    if (!_navigator_ptr->is_ready()) {
        ROS_ERROR("Navigator is not ready!");
        return false;
    }
    ROS_INFO("Routing service is ready.");
    ros::spin();
    return false;
}

} // namespace routing
} // namespace adu

