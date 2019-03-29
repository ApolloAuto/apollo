load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "extention_predictor",
    srcs = ["extention_predictor.cc"],
    hdrs = ["extention_predictor.h"],
    deps = [
        "//modules/prediction/common:prediction_util",
        "//modules/prediction/predictor/sequence:sequence_predictor",
    ],
)

cpplint()
