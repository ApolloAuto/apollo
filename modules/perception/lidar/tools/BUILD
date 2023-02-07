load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "offline_lidar_obstacle_perception",
    srcs = ["offline_lidar_obstacle_perception.cc"],
    deps = [
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common/io:io_util",
        "//modules/perception/common/point_cloud_processing",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/lidar/app",
        "//modules/perception/pipeline/proto:pipeline_config_cc_proto",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_absl//:absl",
        "@eigen",
    ],
)

cpplint()
