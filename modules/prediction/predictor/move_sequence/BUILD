load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "move_sequence_predictor",
    srcs = ["move_sequence_predictor.cc"],
    hdrs = ["move_sequence_predictor.h"],
    deps = [
        "//modules/prediction/predictor",
    ],
)

cc_test(
    name = "move_sequence_predictor_test",
    size = "small",
    srcs = [
        "move_sequence_predictor_test.cc",
    ],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        "//modules/common/configs:config_gflags",
        "//modules/common/util",
        "//modules/prediction/common:kml_map_based_test",
        "//modules/prediction/predictor/move_sequence:move_sequence_predictor",
        "@gtest//:main",
    ],
)


cpplint()