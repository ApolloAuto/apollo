load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "empty_predictor",
    srcs = ["empty_predictor.cc"],
    hdrs = ["empty_predictor.h"],
    deps = [
        "//modules/common:log",
        "//modules/prediction/predictor",
    ],
)

cc_test(
    name = "empty_predictor_test",
    size = "small",
    srcs = [
        "empty_predictor_test.cc",
    ],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        "//modules/common/util",
        "//modules/prediction/common:kml_map_based_test",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/evaluator/vehicle:mlp_evaluator",
        "//modules/prediction/predictor/empty:empty_predictor",
        "//modules/prediction/proto:prediction_proto",
        "@gtest//:main",
    ],
)

cpplint()
