load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "rnn_model",
    srcs = [
        "rnn_model.cc",
    ],
    hdrs = [
        "rnn_model.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common:macro",
        "//modules/prediction/network:net_layer",
        "//modules/prediction/network:net_model",
        "//modules/prediction/proto:network_model_proto",
        "@eigen//:eigen",
    ],
)

cc_test(
    name = "rnn_model_test",
    size = "small",
    srcs = [
        "rnn_model_test.cc",
    ],
    data = [
        "//modules/prediction:prediction_data",
    ],
    deps = [
        "//modules/prediction/network/rnn_model",
        "@gtest//:main",
    ],
)

cpplint()
