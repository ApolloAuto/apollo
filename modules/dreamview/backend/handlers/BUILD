load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])
DREAMVIEW_COPTS = ['-DMODULE_NAME=\\"dreamview\\"']

cc_library(
    name = "websocket_handler",
    srcs = ["websocket_handler.cc"],
    hdrs = ["websocket_handler.h"],
    copts = DREAMVIEW_COPTS,
    deps = [
        "//cyber/common:log",
        "//modules/common/util:map_util",
        "@civetweb//:civetweb++",
        "@com_github_nlohmann_json//:json",
    ],
)

cc_library(
    name = "image_handler",
    srcs = ["image_handler.cc"],
    hdrs = ["image_handler.h"],
    copts = DREAMVIEW_COPTS,
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/configs:config_gflags",
        "//modules/drivers/proto:sensor_image_cc_proto",
        "@civetweb//:civetweb++",
        "@opencv//:imgcodecs",
    ],
)

cc_test(
    name = "websocket_handler_test",
    size = "small",
    srcs = ["websocket_handler_test.cc"],
    deps = [
        ":websocket_handler",
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
