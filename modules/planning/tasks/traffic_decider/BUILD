load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "traffic_rules",
    srcs = [
        "back_side_vehicles.cc",
        "signal_lights.cc",
    ],
    hdrs = [
        "back_side_vehicles.h",
        "signal_lights.h",
        "traffic_rule.h",
    ],
    deps = [
        "//modules/common/adapters:adapter_manager",
        "//modules/common/util:factory",
        "//modules/perception/proto:perception_proto",
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
        "//modules/common/vehicle_state",
        "//modules/planning/common:frame",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/tasks:task",
    ],
)

cpplint()
