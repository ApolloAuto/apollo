load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "speed_decider",
    srcs = ["speed_decider.cc"],
    hdrs = ["speed_decider.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/common/status",
        "//modules/map/proto:map_proto",
        "//modules/planning/common:frame",
        "//modules/planning/common:path_decision",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/tasks:task",
        "//modules/planning/tasks/utils:st_gap_estimator",
    ],
)

cpplint()
