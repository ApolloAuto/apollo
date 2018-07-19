load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "tools_lib",
    srcs = [
        "tools_gflags.cc",
    ],
    hdrs = [
        "tools_gflags.h",
    ],
    linkopts = [
        "-lboost_filesystem",
    ],
    deps = [
        "//modules/drivers/lidar_velodyne/tools/proto:velodyne_tools_proto",
        "//modules/common",
        "//modules/common:apollo_app",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/monitor_log",
        "//modules/common/time",
        "//modules/common/util",
        "@ros//:ros_common",
        "@pcl//:pcl",
    ],
)

cc_library(
    name = "pcd_exporter_lib",
    srcs = [
        "pcd_exporter_app.cc",
        "pcd_exporter.cc",
    ],
    hdrs = [
        "pcd_exporter_app.h",
        "pcd_exporter.h",
    ],
    linkopts = [
        "-lboost_filesystem",
    ],
    deps = [
        ":tools_lib",
        "@ros//:ros_common",
        "@pcl//:pcl",
    ],
)

cc_library(
    name = "pointcloud_tool_lib",
    srcs = [
        "pointcloud_tool.cc",
    ],
    hdrs = [
        "pointcloud_tool.h",
    ],
    linkopts = [
        "-lboost_filesystem",
    ],
    deps = [
        ":tools_lib",
        "//modules/drivers/lidar_velodyne/pointcloud:velodyne_pointcloud",
        "@ros//:ros_common",
        "@pcl//:pcl",
    ],
)



cc_binary(
    name = "pcd_exporter",
    srcs = ["pcd_exporter_main.cc"],
    deps = [
        ":pcd_exporter_lib",
        "//external:gflags",
        "//modules/common",
        "//modules/common/monitor_log",
        "@ros//:ros_common",
    ],
)

cc_binary(
    name = "pointcloud_tool",
    srcs = ["pointcloud_tool_main.cc"],
    deps = [
        ":pointcloud_tool_lib",
        "//external:gflags",
        "//modules/common",
        "@ros//:ros_common",
        "@pcl//:pcl",
    ],
)

cpplint()
