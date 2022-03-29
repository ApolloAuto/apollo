load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

cc_library(
    name = "traffic_rules",
    srcs = [
        # TODO(all): Split to separate targets to gain better building cache.
        "backside_vehicle.cc",
        "crosswalk.cc",
        "destination.cc",
        "keep_clear.cc",
        "reference_line_end.cc",
        "rerouting.cc",
        "stop_sign.cc",
        "traffic_light.cc",
        "yield_sign.cc",
    ],
    hdrs = [
        "backside_vehicle.h",
        "crosswalk.h",
        "destination.h",
        "keep_clear.h",
        "reference_line_end.h",
        "rerouting.h",
        "stop_sign.h",
        "traffic_light.h",
        "traffic_rule.h",
        "yield_sign.h",
    ],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/util:factory",
        "//modules/common/util:map_util",
        "//modules/perception/proto:perception_obstacle_cc_proto",
        "//modules/planning/common:dependency_injector",
        "//modules/planning/common:frame",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/common/util:common_lib",
        "//modules/planning/proto:planning_cc_proto",
        "//modules/planning/proto:planning_config_cc_proto",
        "//modules/planning/proto:traffic_rule_config_cc_proto",
    ],
)

cc_library(
    name = "traffic_decider",
    srcs = ["traffic_decider.cc"],
    hdrs = ["traffic_decider.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":traffic_rules",
        "//modules/common/status",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/planning/common:frame",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/proto:traffic_rule_config_cc_proto",
        "//modules/planning/tasks:task",
    ],
)

cpplint()
