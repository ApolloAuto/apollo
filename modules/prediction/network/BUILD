load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "net_util",
    srcs = ["net_util.cc"],
    hdrs = ["net_util.h"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//cyber",
        "//modules/prediction/proto:network_model_proto",
        "@eigen",
    ],
)

cc_library(
    name = "net_layer",
    srcs = ["net_layer.cc"],
    hdrs = ["net_layer.h"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//modules/prediction/network:net_util",
    ],
)

cc_library(
    name = "net_model",
    srcs = ["net_model.cc"],
    hdrs = ["net_model.h"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//modules/common/util",
        "//modules/prediction/network:net_layer",
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
