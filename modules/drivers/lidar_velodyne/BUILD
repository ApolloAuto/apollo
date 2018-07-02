load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "velodyne_lib",
    srcs = [
        "lidar_velodyne.cc",
    ],
    hdrs = [
        "lidar_velodyne.h",
    ],
    linkopts = [
        "-lboost_system",
        "-lboost_thread",
    ],
    deps = [
        "//modules/common/status",
        "//modules/common:apollo_app",
        "//modules/common",
        "@ros//:ros_common",
        "@pcl//:pcl",
    ],
)

cc_binary(
    name = "lidar_velodyne_tmp",
    srcs = ["main.cc"],
    deps = [
        ":velodyne_lib",
        "//external:gflags",
        "@ros//:ros_common",
    ],
)

cpplint()
