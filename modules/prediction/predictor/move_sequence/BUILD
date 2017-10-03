load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "move_sequence_predictor",
    srcs = ["move_sequence_predictor.cc"],
    hdrs = ["move_sequence_predictor.h"],
    deps = [
        "//modules/prediction/predictor",
        "//modules/common/proto:pnc_point_proto",
        "//modules/prediction/proto:lane_graph_proto",
        "//modules/common/math:kalman_filter",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/common:prediction_util",
        "@eigen//:eigen",
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