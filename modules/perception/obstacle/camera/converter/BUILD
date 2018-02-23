load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_obstacle_camera_converter_geometry",
    srcs = ["geometry_camera_converter.cc"],
    hdrs = ["geometry_camera_converter.h"],
    deps = [

        "//modules/common",
        "//modules/common:log",
        "//modules/perception/common:perception_common",
        "//modules/perception/lib/base",
        "//modules/perception/lib/pcl_util",
        "//modules/perception/obstacle/common:perception_obstacle_common",
        "//modules/perception/obstacle/lidar/interface:perception_obstacle_lidar_interface",
        "@eigen"
        
    ]
)

cpplint()
