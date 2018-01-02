load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "config_manager",
    srcs = ["config_manager.cc"],
    hdrs = ["config_manager.h"],
    deps = [
        "//modules/common:log",
        "//modules/common/util",
        "//modules/perception/common:perception_common",
        "//modules/perception/lib/base",
        "//modules/perception/lib/config_manager/proto:config_proto",
    ],
)

cc_test(
    name = "config_manager_test",
    size = "small",
    srcs = ["config_manager_test.cc"],
    data = ["//modules/perception:perception_data"],
    deps = [
        ":config_manager",
        "//modules/perception/common:perception_common",
        "@gtest//:main",
    ],
)

cpplint()
