load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "object_filter_bank",
    srcs = [
        "object_filter_bank.cc",
    ],
    hdrs = [
        "object_filter_bank.h",
    ],
    deps = [
        "@eigen",
        "//framework:cybertron",
        "//modules/perception/base",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lib/io:protobuf_util",
        "//modules/perception/lib/singleton",
        "//modules/perception/lidar/lib/interface",
        "//modules/perception/lidar/lib/object_filter_bank/proto:filter_bank_config_proto",
        "//modules/perception/proto:perception_config_schema_proto",
    ],
)

cpplint()
