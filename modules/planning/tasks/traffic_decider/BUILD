load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "traffic_rules",
    srcs = [
        "backside_vehicle.cc",
        "clear_zone.cc",
        "crosswalk.cc",
        "object_priority.cc",
        "rerouting.cc",
        "signal_light.cc",
    ],
    hdrs = [
        "backside_vehicle.h",
        "clear_zone.h",
        "crosswalk.h",
        "object_priority.h",
        "rerouting.h",
        "signal_light.h",
        "traffic_rule.h",
    ],
    deps = [
        "//modules/common/adapters:adapter_manager",
        "//modules/common/util:dropbox",
        "//modules/common/util:factory",
        "//modules/perception/proto:perception_proto",
        "//modules/planning/common:frame",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
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
    deps = [
        ":traffic_rules",
        "//modules/common/status",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/planning/common:frame",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/tasks:task",
    ],
)

cpplint()
