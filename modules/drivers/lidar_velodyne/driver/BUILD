load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "velodyne_driver",
    srcs = glob([
        "*.cc",
    ]),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
        "//modules/drivers/lidar_velodyne/common:velodyne_common",
        "//modules/drivers/lidar_velodyne/proto:velodyne_proto",
        "//modules/common",
        "//modules/common/util",
        "@ros//:ros_common",
    ],
)

cpplint()
