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
        "//modules/prediction/proto:network_model_proto",
        "//modules/prediction/network:net_model",
        "//modules/prediction/network:net_layer",
        "//modules/common:log",
        "//modules/common:macro",
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
        "//modules/prediction/network/rnn_model:rnn_model",
        "@gtest//:main",
    ],
)

cpplint()
