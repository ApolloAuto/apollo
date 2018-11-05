load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "junction_predictor",
    srcs = ["junction_predictor.cc"],
    hdrs = ["junction_predictor.h"],
    deps = [
        "//modules/common/proto:pnc_point_proto",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/common:prediction_util",
        "//modules/prediction/predictor",
        "@glog//:glog",
    ],
)

cpplint()
