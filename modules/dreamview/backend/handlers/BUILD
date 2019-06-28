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
    copts = ['-DMODULE_NAME=\\"dreamview\\"'],
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
    copts = ['-DMODULE_NAME=\\"dreamview\\"'],
    linkopts = [
        "-lopencv_core",
        "-lopencv_highgui",
        "-lopencv_imgproc",
    ],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/configs:config_gflags",
        "//modules/drivers/proto:sensor_proto",
        "@civetweb//:civetweb++",
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
        "//cyber",
        "@gtest//:main",
    ],
)

cpplint()
