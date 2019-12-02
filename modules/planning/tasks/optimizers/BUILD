load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "path_optimizer",
    srcs = ["path_optimizer.cc"],
    hdrs = ["path_optimizer.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/status",
        "//modules/planning/common:path_decision",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/common/path:path_data",
        "//modules/planning/common/speed:speed_data",
        "//modules/planning/tasks:task",
        "@eigen",
    ],
)

cc_library(
    name = "speed_optimizer",
    srcs = ["speed_optimizer.cc"],
    hdrs = ["speed_optimizer.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/status",
        "//modules/planning/common:path_decision",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/common:speed_limit",
        "//modules/planning/common:st_graph_data",
        "//modules/planning/common/path:path_data",
        "//modules/planning/common/speed:speed_data",
        "//modules/planning/tasks:task",
        "@eigen",
    ],
)

cc_library(
    name = "trajectory_optimizer",
    srcs = ["trajectory_optimizer.cc"],
    hdrs = ["trajectory_optimizer.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/status",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common/trajectory:discretized_trajectory",
        "//modules/planning/tasks:task",
    ],
)

cpplint()
