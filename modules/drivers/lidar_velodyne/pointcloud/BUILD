load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "velodyne_pointcloud",
    srcs = glob([
        "*.cc",
    ]),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
        "//modules/common",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/util",
        "//modules/drivers/lidar_velodyne/common:velodyne_common",
        "//modules/drivers/lidar_velodyne/proto:velodyne_proto",
        "@pcl",
        "@ros//:ros_common",
        "@yaml_cpp//:yaml",
    ],
)

cpplint()
