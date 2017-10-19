load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "websocket",
    srcs = [
        "websocket.cc",
    ],
    hdrs = [
        "websocket.h",
    ],
    deps = [
        "//modules/common",
        "//modules/common/time",
        "//modules/common/util:string_util",
        "//third_party/json",
        "@civetweb//:civetweb++",
    ],
)

cc_library(
    name = "image",
    srcs = [
        "image.cc",
    ],
    hdrs = [
        "image.h",
    ],
    deps = [
        "//modules/common/adapters:adapter_manager",
        "@civetweb//:civetweb++",
        "@opencv2//:highgui",
    ],
)

cc_test(
    name = "websocket_test",
    size = "small",
    srcs = [
        "websocket_test.cc",
    ],
    deps = [
        ":websocket",
        "//modules/common:log",
        "@gtest//:main",
    ],
)

cpplint()
