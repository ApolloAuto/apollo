load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pointcloud",
    srcs = [
        "calibration.cc",
        "colors.cc",
        "convert.cc",
        "rawdata.cc",
        "transform.cc",
    ],
    hdrs = [
        "calibration.h",
        "cloud_nodelet.h",
        "colors.h",
        "convert.h",
        "point_types.h",
        "rawdata.h",
        "ringcolors_nodelet.h",
        "transform.h",
        "transform_nodelet.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/drivers/lidar_velodyne/proto:cloud_node_conf_proto",
        "//modules/drivers/lidar_velodyne/proto:transform_node_conf_proto",
        "@eigen",
        "@pcl",
        "@ros//:ros_common",
        "@yaml_cpp//:yaml",
    ],
)

cpplint()
