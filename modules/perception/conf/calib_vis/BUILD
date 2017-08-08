load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "calib_vis_lib",
    srcs = ["calib_vis.cc"],
    hdrs = ["calib_vis.h"],
    deps = [
        "//modules/common:apollo_app",
        "//modules/common:log",
        "//modules/common/adapters:adapter_manager",
        "//modules/perception/conf/calib_vis/common:calib_vis_common",
        "//modules/perception/lib/pcl_util",
        "//modules/localization/proto:gps_proto",
        "@vtk//:vtk",
        "@yaml_cpp//:yaml",
    ],
)

cc_binary(
    name = "calib_vis",
    srcs = ["main.cc"],
    data = [ 
        "//modules/perception/conf/calib_vis/conf:calib_vis_adapter_manager_config"
    ],
    deps = [
        ":calib_vis_lib",
        "//external:gflags",
        "//modules/common:log",
        "@ros//:ros_common",
    ],
    linkstatic = 0,
)

cpplint()
