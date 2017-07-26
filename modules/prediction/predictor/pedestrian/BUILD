load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "regional_predictor",
    hdrs = ["regional_predictor.h"],
    srcs = ["regional_predictor.cc"],
    deps = [
        "//modules/prediction/predictor:predictor",
        "//modules/prediction/common:prediction_util",
        "//modules/prediction/common:prediction_gflags",
        "//modules/common/proto:path_point_proto",
        "//modules/common/math:kalman_filter",
        "@glog//:glog",
    ]
)

cpplint()