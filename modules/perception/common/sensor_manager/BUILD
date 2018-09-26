load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "sensor_manager",
    srcs = [
        "sensor_manager.cc",
    ],
    hdrs = [
        "sensor_manager.h",
    ],
    deps = [
        "//framework:cybertron",
        "//modules/perception/base:base_type",
        "//modules/perception/base:camera",
        "//modules/perception/base:omnidirectional_model",
        "//modules/perception/base:polynomial",
        "//modules/perception/common/io:io_util",
        "//modules/perception/lib/config_manager:config_manager",
        "//modules/perception/lib/io:file_util",
        "//modules/perception/lib/thread:thread",
        "//modules/perception/proto:sensor_meta_schema_proto",
        "@yaml_cpp//:yaml",
    ],
)

cc_test(
    name = "sensor_manager_test",
    size = "small",
    srcs = [
        "sensor_manager_test.cc",
    ],
    deps = [
        ":sensor_manager",
        "@gtest//:main",
    ],
)

cpplint()
