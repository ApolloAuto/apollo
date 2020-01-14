load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "object_filter_bank",
    srcs = ["object_filter_bank.cc"],
    hdrs = ["object_filter_bank.h"],
    deps = [
        "//cyber",
        "//modules/common/util",
        "//modules/perception/base",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lidar/lib/interface",
        "//modules/perception/lidar/lib/object_filter_bank/proto:filter_bank_config_proto",
        "//modules/perception/proto:perception_config_schema_proto",
        "@eigen",
    ],
)

cc_test(
    name = "object_filter_bank_test",
    size = "small",
    srcs = ["object_filter_bank_test.cc"],
    deps = [
        ":object_filter_bank",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
