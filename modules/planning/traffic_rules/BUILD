load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "traffic_rules",
    srcs = [
        # TODO(all): Split to separate targets to gain better building cache.
        "backside_vehicle.cc",
        "change_lane.cc",
        "crosswalk.cc",
        "destination.cc",
        "keep_clear.cc",
        "object_priority.cc",
        "reference_line_end.cc",
        "rerouting.cc",
        "stop_sign.cc",
        "traffic_light.cc",
    ],
    hdrs = [
        "backside_vehicle.h",
        "change_lane.h",
        "crosswalk.h",
        "destination.h",
        "keep_clear.h",
        "object_priority.h",
        "reference_line_end.h",
        "rerouting.h",
        "stop_sign.h",
        "traffic_light.h",
        "traffic_rule.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/util:factory",
        "//modules/common/util:map_util",
        "//modules/perception/proto:perception_proto",
        "//modules/planning/common:frame",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/common/util:common_lib",
        "//modules/planning/proto:planning_config_proto",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/proto:traffic_rule_config_proto",
    ],
)

cc_library(
    name = "traffic_decider",
    srcs = [
        "traffic_decider.cc",
    ],
    hdrs = [
        "traffic_decider.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":traffic_rules",
        "//modules/common/status",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/planning/common:frame",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/proto:traffic_rule_config_proto",
        "//modules/planning/tasks:task",
    ],
)

cpplint()
