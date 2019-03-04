load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "speed_bounds_decider",
    srcs = [
        "speed_bounds_decider.cc",
    ],
    hdrs = [
        "speed_bounds_decider.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/status",
        "//modules/planning/common:frame",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:st_graph_data",
        "//modules/planning/tasks/deciders:decider_base",
        "//modules/planning/tasks:task",
        "//modules/planning/tasks/optimizers/st_graph:st_boundary_mapper",
    ],
)

cpplint()
