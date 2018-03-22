load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "adc_trajectory_container",
    srcs = ["adc_trajectory_container.cc"],
    hdrs = ["adc_trajectory_container.h"],
    deps = [
        "//modules/common:log",
        "//modules/common/math:geometry",
        "//modules/planning/proto:planning_proto",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/common:prediction_map",
        "//modules/prediction/container",
        "//modules/prediction/proto:lane_graph_proto",
    ],
)

cpplint()
