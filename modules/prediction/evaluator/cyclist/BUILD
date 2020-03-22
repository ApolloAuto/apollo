load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "cyclist_keep_lane_evaluator",
    srcs = ["cyclist_keep_lane_evaluator.cc"],
    hdrs = ["cyclist_keep_lane_evaluator.h"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/evaluator",
    ],
)

cc_test(
    name = "cyclist_keep_lane_evaluator_test",
    size = "small",
    srcs = ["cyclist_keep_lane_evaluator_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        ":cyclist_keep_lane_evaluator",
        "//modules/prediction/common:kml_map_based_test",
        "//modules/prediction/container/obstacles:obstacles_container",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
