load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])
PREDICTION_COPTS = ["-DMODULE_NAME=\\\"prediction\\\""]

cc_library(
    name = "net_util",
    srcs = ["net_util.cc"],
    hdrs = ["net_util.h"],
    copts = PREDICTION_COPTS,
    deps = [
        "//cyber/common:log",
        "//modules/prediction/proto:network_layers_cc_proto",
        "@eigen",
    ],
)

cc_library(
    name = "net_layer",
    srcs = ["net_layer.cc"],
    hdrs = ["net_layer.h"],
    copts = PREDICTION_COPTS,
    deps = [
        "//modules/prediction/network:net_util",
    ],
)

cc_library(
    name = "net_model",
    srcs = ["net_model.cc"],
    hdrs = ["net_model.h"],
    copts = PREDICTION_COPTS,
    deps = [
        "//modules/common/util",
        "//modules/prediction/network:net_layer",
        "//modules/prediction/proto:network_model_cc_proto",
    ],
)

cc_test(
    name = "net_util_test",
    size = "small",
    srcs = ["net_util_test.cc"],
    deps = [
        "//modules/prediction/network:net_util",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "net_layer_test",
    size = "small",
    srcs = ["net_layer_test.cc"],
    deps = [
        "//modules/prediction/network:net_layer",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
