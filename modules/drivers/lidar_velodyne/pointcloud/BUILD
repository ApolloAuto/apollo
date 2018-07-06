load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "compensator",
    srcs = [
        "compensator.cc",
        "compensator_nodelet.cc",
    ],
    hdrs = [
        "compensator.h",
    ],
    deps = [
        "//modules/common",
        "//modules/drivers/lidar_velodyne/pointcloud/lib:util",
        "@ros//:ros_common",
        "@eigen",
        "@pcl",
    ],
)

cc_library(
    name = "convert",
    srcs = [
        "convert.cc",
        "convert_nodelet.cc",
    ],
    hdrs = [
        "convert.h",
    ],
    deps = [
        "//modules/common",
        "//modules/drivers/lidar_velodyne/pointcloud/lib:util",
        "//modules/drivers/lidar_velodyne/pointcloud/lib:velodyne_parser",
        "@ros//:ros_common",
        "@eigen",
        "@pcl",
    ],
)

cc_library(
    name = "pcd_exporter",
    srcs = [
        "pcd_exporter.cc",
        "pcd_exporter_nodelet.cc",
    ],
    hdrs = [
        "pcd_exporter.h",
    ],
    deps = [
        "//modules/common",
        "//modules/drivers/lidar_velodyne/pointcloud/lib:util",
        "@ros//:ros_common",
        "@eigen",
        "@pcl",
    ],
)

cc_library(
    name = "pointcloud_dump",
    srcs = [
        "pointcloud_dump.cc",
        "pointcloud_dump_nodelet.cc",
    ],
    hdrs = [
        "pointcloud_dump.h",
    ],
    deps = [
        "//modules/common",
        "//modules/drivers/lidar_velodyne/pointcloud/lib:util",
        "//modules/drivers/lidar_velodyne/pointcloud/lib:velodyne_parser",
        "@ros//:ros_common",
        "@eigen",
        "@pcl",
    ],
)

cpplint()
