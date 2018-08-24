load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "usb_cam_nodes",
    srcs = [
        "usb_cam_wrapper.cc",
    ],
    hdrs = [
        "camera_nodelet.h",
        "usb_cam_wrapper.h",
    ],
    deps = [
        "//modules/common",
        "//modules/common:log",
        "//modules/common/util",
        "//modules/drivers/proto:sensor_proto",
        "//modules/drivers/camera/common:camera_common",
        "//modules/drivers/camera/proto:camero_proto",
        "//modules/drivers/camera/src:usb_cam_lib",
        "@ros//:ros_common",
        "@yaml_cpp//:yaml",
    ],
)

# cpplint()
