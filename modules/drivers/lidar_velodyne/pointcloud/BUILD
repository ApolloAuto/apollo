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

cpplint()
