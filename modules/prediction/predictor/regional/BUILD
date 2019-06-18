load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "regional_predictor",
    srcs = ["regional_predictor.cc"],
    hdrs = ["regional_predictor.h"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//modules/prediction/common:prediction_util",
        "//modules/prediction/predictor",
    ],
)

cc_test(
    name = "regional_predictor_test",
    size = "small",
    srcs = [
        "regional_predictor_test.cc",
    ],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        "//modules/prediction/common:kml_map_based_test",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/predictor/regional:regional_predictor",
        "@gtest//:main",
    ],
)

cpplint()
