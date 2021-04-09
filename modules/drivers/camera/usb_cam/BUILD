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
        "@ros//:ros_common",
    ]  + select({
        "//tools/platforms:aarch64": [],
        "//conditions:default": [
            "@adv_plat//:adv_plat",
        ],
    }) ,
)

# cpplint()
