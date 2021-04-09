load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lidar_ex_checker_lib",
    srcs = ["lidar_ex_checker.cc"],
    hdrs = ["lidar_ex_checker.h"],
    copts = [
        "-Wno-deprecated",
    ],
    deps = [
        "//modules/calibration/lidar_ex_checker/common:lidar_ex_checker_common",
        "//modules/common:apollo_app",
        "//modules/common:log",
        "//modules/common/adapters:adapter_manager",
        "//modules/localization/proto:gps_proto",
        "//modules/perception/common:pcl_util",
        "@vtk",
    ],
)

cc_binary(
    name = "lidar_ex_checker",
    srcs = ["main.cc"],
    data = [
        "//modules/calibration/lidar_ex_checker/conf:lidar_ex_checker_adapter_manager_config",
    ],
    deps = [
        ":lidar_ex_checker_lib",
        "//external:gflags",
        "//modules/common:log",
        "@pcl",
        "@ros//:ros_common",
    ],
)

cpplint()
