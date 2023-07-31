load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "object_filter_bank",
    srcs = ["object_filter_bank.cc"],
    hdrs = ["object_filter_bank.h"],
    deps = [
        "//cyber",
        "//modules/common/util",
        "//modules/perception/common:util",
        "//modules/perception/common/base",
        "//modules/perception/common/lidar/common",
        "//modules/perception/common/proto:perception_config_schema_cc_proto",
        "//modules/perception/lidar_detection_filter/interface:base_object_filter",
        "//modules/perception/lidar_detection_filter/object_filter_bank/proto:filter_bank_config_cc_proto",
        "//modules/perception/common/lib/interface:base_init_options",
    ],
)

# cc_test(
#     name = "object_filter_bank_test",
#     size = "small",
#     srcs = ["object_filter_bank_test.cc"],
#     deps = [
#         ":object_filter_bank",
#         "@com_google_googletest//:gtest_main",
#     ],
# )

apollo_package()
cpplint()
