load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "traffic_decider",
    srcs = [
        "traffic_decider.cc",
    ],
    hdrs = [
        "traffic_decider.h",
    ],
    deps = [
        "//modules/common/status",
        "//modules/common/vehicle_state",
        "//modules/planning/common:frame",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/tasks:task",
    ],
)

cpplint()
