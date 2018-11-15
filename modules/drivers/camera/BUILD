load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "camera_lib",
    srcs = [
        "camera.cc",
    ],
    hdrs = [
        "camera.h",
    ],
    linkopts = [
        "-lboost_system",
        "-lboost_thread",
    ],
    deps = [
        "//modules/common",
        "//modules/common:apollo_app",
        "//modules/common:log",
        "//modules/common/adapters:adapter",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/monitor_log",
        "//modules/common/proto:error_code_proto",
        "//modules/common/status",
        "//modules/drivers/camera/nodes:camera_nodelet",
        "//modules/drivers/camera/nodes:usb_cam_wrapper",
        "//modules/drivers/camera/proto:camera_proto",
        "@pcl",
        "@ros//:ros_common",
    ],
)

cc_binary(
    name = "camera",
    srcs = [
        "main.cc",
    ],
    deps = [
        ":camera_lib",
        "//external:gflags",
    ],
)

cpplint()
