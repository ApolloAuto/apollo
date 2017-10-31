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
        "//modules/prediction/evaluator/network:net_model",
        "//modules/prediction/evaluator/network/layer:layer",
        "//modules/common:log",
        "//modules/common:macro",
        "@eigen//:eigen",
    ],
)

cc_library(
    name = "net_model",
    srcs = [
        "net_model.cc",
    ],
    hdrs = [
        "net_model.h",
    ],
    deps = [
        "//modules/prediction/proto:network_model_proto",
        "//modules/prediction/evaluator/network/layer:layer",
        "//modules/common:log",
        "//modules/common/util",
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
        "//modules/prediction/evaluator/network:rnn_model",
        "@gtest//:main",
    ],
)


cpplint()
