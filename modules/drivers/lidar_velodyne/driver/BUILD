load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "driver",
    srcs = [
        "driver.cc",
        "input.cc",
    ],
    hdrs = [
        "driver.h",
        "driver_nodelet.h",
        "input.h",
        "ring_sequence.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common/util",
        "//modules/drivers/lidar_velodyne/proto:driver_node_conf_proto",
        "@ros//:ros_common",
    ],
)

cpplint()
