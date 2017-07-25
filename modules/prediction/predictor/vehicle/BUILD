load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lane_sequence_predictor",
    hdrs = ["lane_sequence_predictor.h"],
    srcs = ["lane_sequence_predictor.cc"],
    deps = [
        "//modules/prediction/predictor:predictor",
        "//modules/common:log",
    ]
)

cc_library(
    name = "free_move_predictor",
    hdrs = ["free_move_predictor.h"],
    srcs = ["free_move_predictor.cc"],
    deps = [
        "//modules/prediction/predictor:predictor",
        "//modules/prediction/proto:feature_proto",
        "//modules/prediction/common:prediction_gflags",
        "//modules/common:log",
        "@eigen//:eigen",
    ]
)

cpplint()