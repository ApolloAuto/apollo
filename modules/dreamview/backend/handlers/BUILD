load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "websocket_handler",
    srcs = ["websocket_handler.cc"],
    hdrs = ["websocket_handler.h"],
    copts = ['-DMODULE_NAME=\\"dreamview\\"'],
    deps = [
        "//modules/common/time",
        "//modules/common/util:map_util",
        "@civetweb//:civetweb++",
        "@com_github_nlohmann_json//:json",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "image_handler",
    srcs = ["image_handler.cc"],
    hdrs = ["image_handler.h"],
    copts = ['-DMODULE_NAME=\\"dreamview\\"'],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/configs:config_gflags",
        "//modules/drivers/proto:sensor_image_cc_proto",
        "@civetweb//:civetweb++",
        "@opencv",
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
