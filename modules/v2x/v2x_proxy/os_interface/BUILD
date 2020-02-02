load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "os_interface",
    srcs = ["os_interface.cc"],
    hdrs = ["os_interface.h"],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/localization/proto:localization_proto",
        "//modules/perception/proto:perception_proto",
        "//modules/v2x/common:v2x_proxy_gflags",
        "//modules/v2x/proto:v2x_traffic_light_proto",
    ],
)

cc_test(
    name = "os_interface_test",
    size = "medium",
    timeout = "moderate",
    srcs = ["os_interface_test.cc"],
    deps = [
        ":os_interface",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
