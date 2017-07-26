load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "regional_predictor",
    srcs = ["regional_predictor.cc"],
    hdrs = ["regional_predictor.h"],
    deps = [
        "//modules/common/math:kalman_filter",
        "//modules/common/proto:path_point_proto",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/common:prediction_util",
        "//modules/prediction/predictor",
        "@glog//:glog",
    ],
)

cpplint()
