load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "adc_trajectory_container",
    srcs = ["adc_trajectory_container.cc"],
    hdrs = ["adc_trajectory_container.h"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//modules/planning/proto:planning_proto",
        "//modules/prediction/common:prediction_map",
        "//modules/prediction/container",
    ],
)

cc_test(
    name = "adc_trajectory_container_test",
    size = "small",
    srcs = ["adc_trajectory_container_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        ":adc_trajectory_container",
        "//modules/prediction/common:kml_map_based_test",
        "@gtest//:main",
    ],
)

cpplint()
