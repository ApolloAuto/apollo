load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lane_sequence_predictor",
    srcs = ["lane_sequence_predictor.cc"],
    hdrs = ["lane_sequence_predictor.h"],
    deps = [
        "//modules/common:log",
        "//modules/common/math:math_utils",
        "//modules/common/proto:pnc_point_proto",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/common:prediction_map",
        "//modules/prediction/predictor",
        "//modules/prediction/proto:lane_graph_proto",
        "@eigen//:eigen",
    ],
)

cc_library(
    name = "free_move_predictor",
    srcs = ["free_move_predictor.cc"],
    hdrs = ["free_move_predictor.h"],
    deps = [
        "//modules/common:log",
        "//modules/common/math:math_utils",
        "//modules/common/proto:pnc_point_proto",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/common:prediction_util",
        "//modules/prediction/predictor",
        "//modules/prediction/proto:feature_proto",
        "@eigen//:eigen",
    ],
)

cpplint()
