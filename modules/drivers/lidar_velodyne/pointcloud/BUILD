load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "velodyne_pointcloud",
    srcs = [
        "calibration.cc",
        "rawdata.cc",
    ],
    hdrs = [
        "calibration.h",
        "point_types.h",
        "rawdata.h",
    ],
    deps = [
        "@eigen",
        "@pcl",
        "@ros//:ros_common",
        "@yaml_cpp//:yaml",
    ],
)

cpplint()
