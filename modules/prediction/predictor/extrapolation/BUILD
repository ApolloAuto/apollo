load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "extrapolation_predictor",
    srcs = ["extrapolation_predictor.cc"],
    hdrs = ["extrapolation_predictor.h"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//modules/prediction/common:prediction_util",
        "//modules/prediction/predictor/sequence:sequence_predictor",
    ],
)

cpplint()
