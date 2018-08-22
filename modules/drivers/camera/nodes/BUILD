load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "usb_cam_nodes",
    srcs = [
        "usb_cam_wrapper.cc",
        "usb_cam_nodelet.cc",
    ],
    hdrs = [
        "usb_cam_wrapper.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/drivers/proto:sensor_proto",
        "//modules/drivers/camera/src:usb_cam_lib",
        "@ros//:ros_common",
        "@yaml_cpp//:yaml",
    ],
)

# cpplint()
