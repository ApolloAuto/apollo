load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "config_manager",
    srcs = ["config_manager.cc",],
    hdrs = ["config_manager.h",],
    deps = [
        "//modules/common:log",
        "//modules/perception/lib/base:base",
        "//modules/perception/lib/config_manager/proto:config_proto",
        "//modules/perception/common:perception_common",
    ],
)

cc_test(
    name = "config_manager_test",
    srcs = ["config_manager_test.cc",],
    size = "small",
    data = ["//modules/perception:perception_testdata"],
    deps = [
        ":config_manager",
        "//modules/perception/common:perception_common",
        "@gtest//:main",
    ],
)

cc_test(
    name = "config_manager_test",
    srcs = ["config_manager_test.cc",],
    size = "small",
    data = ["//modules/perception:perception_testdata"],
    deps = [
        ":config_manager",
        "@gtest//:main",
    ],
)

cpplint()
