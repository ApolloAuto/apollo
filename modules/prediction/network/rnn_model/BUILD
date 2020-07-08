load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "rnn_model",
    srcs = ["rnn_model.cc"],
    hdrs = ["rnn_model.h"],
    deps = [
        "//cyber/base:macros",
        "//cyber/common:macros",
        "//modules/prediction/network:net_model",
    ],
)

cc_test(
    name = "rnn_model_test",
    size = "small",
    srcs = ["rnn_model_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
    ],
    deps = [
        "//cyber/common:file",
        "//modules/prediction/network/rnn_model",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
