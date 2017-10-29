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
        "//modules/prediction/evaluator/network:layer",
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
        "//modules/prediction/evaluator/network:layer",
        "//modules/common:log",
        "@eigen//:eigen",
    ],
)

cc_library(
    name = "layer",
    srcs = [
        "layer.cc",
    ],
    hdrs = [
        "layer.h",
    ],
    deps = [
        "//modules/prediction/proto:network_model_proto",
        "//modules/prediction/evaluator/network:util",
        "//modules/common:log",
        "@eigen//:eigen",
    ],
)

cc_library(
    name = "util",
    srcs = [
        "util.cc",
    ],
    hdrs = [
        "util.h",
    ],
    deps = [
        "//modules/prediction/proto:network_model_proto",
        "//modules/common:log",
        "@eigen//:eigen",
    ],
)

cc_test(
    name = "rnn_model_test",
    size = "small",
    srcs = [
        "rnn_model_test.cc",
    ],
    deps = [
        "//modules/prediction/evaluator/network:rnn_model",
        "@gtest//:main",
    ],
)

cc_test(
    name = "layer_test",
    size = "small",
    srcs = [
        "layer_test.cc",
    ],
    deps = [
        "//modules/prediction/evaluator/network:util",
        "//modules/prediction/evaluator/network:layer",
        "//modules/prediction/proto:network_model_proto",
        "@eigen//:eigen",
        "@gtest//:main",
    ],
)

cc_test(
    name = "util_test",
    size = "small",
    srcs = [
        "util_test.cc",
    ],
    deps = [
        "//modules/prediction/evaluator/network:util",
        "//modules/prediction/proto:network_model_proto",
        "@eigen//:eigen",
        "@gtest//:main",
    ],
)

cpplint()
