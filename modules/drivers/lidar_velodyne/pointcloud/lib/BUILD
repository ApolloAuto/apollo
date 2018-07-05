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
    name = "calibration",
    srcs = [
        "online_calibration.cc",
    ],
    hdrs = [
        "calibration.h",
        "online_calibration.h",
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
        "velodyne16_parser.cc",
        "velodyne32_parser.cc",
        "velodyne64_parser.cc",
    ],
    hdrs = [
        "const_variables.h",
        "point_types.h",
        "velodyne_parser.h",
    ],
    deps = [
        ":calibration",
        ":util",
        "@ros//:ros_common",
        "@pcl",
        "@eigen",
    ],
)

cpplint()
