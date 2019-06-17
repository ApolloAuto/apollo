load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "junction_predictor",
    srcs = ["junction_predictor.cc"],
    hdrs = ["junction_predictor.h"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//modules/prediction/common:prediction_util",
        "//modules/prediction/predictor",
    ],
)

cc_test(
    name = "junction_predictor_test",
    size = "small",
    srcs = [
        "junction_predictor_test.cc",
    ],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        "//modules/prediction/common:kml_map_based_test",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/evaluator/vehicle:junction_mlp_evaluator",
        "//modules/prediction/predictor/junction:junction_predictor",
        "@gtest//:main",
    ],
)

cpplint()
