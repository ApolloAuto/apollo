load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "driver",
    srcs = [
        "driver.cc",
        "driver16.cc",
        "driver32.cc",
        "driver64.cc",
    ],
    hdrs = [
        "driver.h",
    ],
    deps = [
        "//modules/common",
        "//modules/drivers/lidar_velodyne/common",
        "@ros//:ros_common",
    ],
)

cpplint()
