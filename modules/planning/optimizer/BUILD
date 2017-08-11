load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "optimizer",
    srcs = [
        "optimizer.cc",
    ],
    hdrs = [
        "optimizer.h",
    ],
    deps = [
        "//modules/common/status",
        "//modules/planning/common:frame",
        "//modules/planning/common/trajectory:discretized_trajectory",
        "//modules/planning/common/trajectory:publishable_trajectory",
    ],
)

cc_library(
    name = "path_optimizer",
    srcs = [
        "path_optimizer.cc",
    ],
    hdrs = [
        "path_optimizer.h",
    ],
    deps = [
        ":optimizer",
        "//modules/common/status",
        "//modules/planning/common:path_decision",
        "//modules/planning/common:planning_data",
        "//modules/planning/common/path:path_data",
        "//modules/planning/common/speed:speed_data",
        "@eigen//:eigen",
    ],
)

cc_library(
    name = "speed_optimizer",
    srcs = [
        "speed_optimizer.cc",
    ],
    hdrs = [
        "speed_optimizer.h",
    ],
    deps = [
        ":optimizer",
        "//modules/common/status",
        "//modules/planning/common:path_decision",
        "//modules/planning/common:planning_data",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:speed_limit",
        "//modules/planning/common/path:path_data",
        "//modules/planning/common/speed:speed_data",
        "//modules/planning/optimizer/st_graph:st_graph_boundary",
        "//modules/planning/optimizer/st_graph:st_graph_point",
        "@eigen//:eigen",
    ],
)

cpplint()
