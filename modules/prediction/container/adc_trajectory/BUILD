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

cc_test(
    name = "adc_trajectory_container_test",
    size = "small",
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    srcs = ["adc_trajectory_container_test.cc"],
    deps = [
        ":adc_trajectory_container",
        "//modules/prediction/common:kml_map_based_test",
        "@gtest//:main",
    ],
)

cpplint()
