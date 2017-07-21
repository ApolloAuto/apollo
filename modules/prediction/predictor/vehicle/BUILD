load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lane_sequence_predictor",
    hdrs = ["lane_sequence_predictor.h"],
    srcs = ["lane_sequence_predictor.cc"],
    deps = [
        "//modules/prediction/predictor:predictor",
        "//modules/prediction/proto:prediction_proto",
        "@glog//:glog",
    ]
)

cpplint()