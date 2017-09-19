load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "traffic_rules",
    srcs = [
        "back_side_vehicles.cc",
    ],
    hdrs = [
        "back_side_vehicles.h",
        "traffic_rule.h",
    ],
    deps = [
        "//modules/common/util:factory",
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
