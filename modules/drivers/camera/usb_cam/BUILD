load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "usb_cam_lib",
    srcs = [
        "usb_cam.cc",
    ],
    hdrs = [
        "usb_cam.h",
    ],
    deps = [
        "//modules/common:log",
        "@adv_plat//:adv_plat",
        "@ros//:ros_common",
    ],
)

# cpplint()
