load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "util",
    srcs = [
        "util.cc",
    ],
    hdrs = [
        "util.h",
    ],
    deps = [
        "//modules/common",
        "@ros//:ros_common",
    ],
)

cc_library(
    name = "velodyne_parser",
    srcs = [
        "velodyne_parser.cc",
    ],
    hdrs = [
        "calibration.h",
        "const_variables.h",
        "online_calibration.h",
        "point_types.h",
        "velodyne_parser.h",
    ],
    deps = [
        ":util",
        "@ros//:ros_common",
        "@pcl",
        "@eigen",
    ],
)
cpplint()
