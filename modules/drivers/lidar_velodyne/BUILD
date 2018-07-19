load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "velodyne_lib",
    srcs = [
        "velodyne.cc",
    ],
    hdrs = [
        "velodyne.h",
    ],
    linkopts = [
        "-lboost_system",
        "-lboost_thread",
    ],
    deps = [
        "//modules/common",
        "//modules/common:apollo_app",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/monitor_log",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/drivers/lidar_velodyne/common:velodyne_common",
        "//modules/drivers/lidar_velodyne/driver:velodyne_driver",
        "//modules/drivers/lidar_velodyne/pointcloud:velodyne_pointcloud",
        "//modules/drivers/lidar_velodyne/proto:velodyne_proto",
        "@pcl",
        "@ros//:ros_common",
    ],
)

cc_binary(
    name = "lidar_velodyne",
    srcs = ["main.cc"],
    deps = [
        ":velodyne_lib",
        "//external:gflags",
        "//modules/common",
        "//modules/common/monitor_log",
        "@ros//:ros_common",
    ],
)

cpplint()
