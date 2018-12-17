load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "offline_lidar_obstacle_perception",
    srcs = ["offline_lidar_obstacle_perception.cc"],
    deps = [
        "//cyber",
        "//modules/drivers/proto:sensor_proto",
        "//modules/map/proto:map_proto",
        "//modules/perception/base",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common/io:io_util",
        "//modules/perception/common/point_cloud_processing",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/lidar/app",
        "//modules/perception/lidar/lib/object_builder",
        "//modules/perception/proto:perception_config_schema_proto",
        "@eigen",
        "@pcl",
    ],
)

cpplint()
