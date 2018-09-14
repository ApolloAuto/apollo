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
        "//framework:cybertron",
        "//modules/common/monitor_log",
        "//modules/common/proto:error_code_proto",
        "//modules/common/status",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/drivers/lidar_velodyne/common:velodyne_gflags",
        "//modules/drivers/lidar_velodyne/driver",
        "//modules/drivers/lidar_velodyne/pointcloud",
        "@pcl",
        "@ros//:ros_common",
    ],
)

cc_binary(
    name = "lidar_velodyne",
    srcs = [
        "main.cc",
    ],
    deps = [
        ":velodyne_lib",
        "//external:gflags",
    ],
)

cpplint()
