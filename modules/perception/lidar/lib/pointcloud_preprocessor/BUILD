load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pointcloud_preprocessor",
    srcs = [
        "pointcloud_preprocessor.cc",
    ],
    hdrs = [
        "pointcloud_preprocessor.h",
    ],
    deps = [
        "//framework:cybertron",
        "//modules/common/proto:error_code_proto",
        "//modules/common/proto:header_proto",
        "//modules/common/util",
        "//modules/drivers/proto:sensor_proto",
        "//modules/perception/base",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lidar/common",
        "//modules/perception/lidar/lib/pointcloud_preprocessor/proto:pointcloud_preprocessor_config_proto",
        "//modules/perception/proto:perception_config_schema_proto",
        "@eigen",
    ],
)

cc_test(
    name = "pointcloud_preprocessor_test",
    size = "small",
    srcs = [
        "pointcloud_preprocessor_test.cc",
    ],
    deps = [
        ":pointcloud_preprocessor",
        "@gtest//:main",
        "//modules/perception/base",
        "//modules/perception/common/io:io_util",
        "//modules/perception/lidar/common",
        "//modules/perception/lidar/lib/interface",
        "//modules/perception/common/graph",
        "//modules/perception/lidar/lib/object_builder",
        "//modules/perception/lidar/lib/tracker/common:track_pool_types",
    ],
)

cpplint()
