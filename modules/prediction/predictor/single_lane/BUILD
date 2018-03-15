load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "single_lane_predictor",
    srcs = ["single_lane_predictor.cc"],
    hdrs = ["single_lane_predictor.h"],
    deps = [
        "//modules/common:log",
        "//modules/common/adapters/proto:adapter_config_proto",
        "//modules/common/math",
        "//modules/common/proto:pnc_point_proto",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/common:prediction_map",
        "//modules/prediction/common:prediction_util",
        "//modules/prediction/common:validation_checker",
        "//modules/prediction/predictor/sequence:sequence_predictor",
        "//modules/prediction/proto:lane_graph_proto",
        "@eigen",
    ],
)

cpplint()
