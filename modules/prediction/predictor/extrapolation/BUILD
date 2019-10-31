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
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/common:prediction_map",
        "//modules/prediction/common:prediction_util",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/predictor/sequence:sequence_predictor",
    ],
)

cpplint()
