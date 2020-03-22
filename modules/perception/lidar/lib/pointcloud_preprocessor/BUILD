load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pointcloud_preprocessor",
    srcs = ["pointcloud_preprocessor.cc"],
    hdrs = ["pointcloud_preprocessor.h"],
    deps = [
        "//cyber",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/configs/proto:vehicle_config_proto",
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

cpplint()
