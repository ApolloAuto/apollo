load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "usb_cam_wrapper",
    srcs = [
        "usb_cam_wrapper.cc",
    ],
    hdrs = [
        "usb_cam_wrapper.h",
    ],
    deps = [
        "//modules/common",
        "//modules/common:log",
        "//modules/common/util",
        "//modules/drivers/proto:sensor_proto",
        "//modules/drivers/camera/common:camera_common",
        "//modules/drivers/camera/proto:camera_proto",
        "//modules/drivers/camera/usb_cam:usb_cam_lib",
        "@ros//:ros_common",
        "@yaml_cpp//:yaml",
    ],
)

cc_library(
    name = "camera_nodelet",
    srcs = [
        "camera_nodelet.cc",
    ],
    hdrs = [
        "camera_nodelet.h",
    ],
    deps = [
        "//modules/drivers/camera/nodes:usb_cam_wrapper",
    ],
)


# cpplint()
