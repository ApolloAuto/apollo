load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

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
        "//modules/prediction/evaluator/network/layer:util",
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
    name = "layer_test",
    size = "small",
    srcs = [
        "layer_test.cc",
    ],
    deps = [
        "//modules/prediction/evaluator/network/layer:util",
        "//modules/prediction/evaluator/network/layer:layer",
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
        "//modules/prediction/evaluator/network/layer:util",
        "//modules/prediction/proto:network_model_proto",
        "@eigen//:eigen",
        "@gtest//:main",
    ],
)

cpplint()
