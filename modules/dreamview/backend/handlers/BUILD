load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "websocket_handler",
    srcs = [
        "websocket_handler.cc",
    ],
    hdrs = [
        "websocket_handler.h",
    ],
    deps = [
        "//modules/common",
        "//modules/common/time",
        "//modules/common/util:map_util",
        "//modules/common/util:string_util",
        "//third_party/json",
        "@civetweb//:civetweb++",
    ],
)

cc_library(
    name = "image_handler",
    srcs = [
        "image_handler.cc",
    ],
    hdrs = [
        "image_handler.h",
    ],
    deps = [
        "//modules/common/adapters:adapter_manager",
        "//modules/perception/traffic_light/util",
        "@civetweb//:civetweb++",
        "@opencv2//:highgui",
    ],
)

cc_test(
    name = "websocket_handler_test",
    size = "small",
    srcs = [
        "websocket_handler_test.cc",
    ],
    deps = [
        ":websocket_handler",
        "//modules/common:log",
        "@gtest//:main",
    ],
)

cpplint()
