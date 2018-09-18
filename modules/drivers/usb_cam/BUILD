load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libusb_cam_component.so",
    deps = [":usb_cam_component_lib"],
    linkopts = [
       "-shared",
       "-lswscale",
       "-lavformat",
       "-lavcodec",
    ],
    linkstatic = False,
)

cc_library(
    name = "usb_cam_component_lib",
    srcs = ["usb_cam_component.cc"],
    hdrs = ["usb_cam_component.h"],
    deps = [
        ":usb_cam",
        "//modules/drivers/proto:sensor_proto",
        "//framework:cybertron",
    ],
    copts = ['-DMODULE_NAME=\\"usb_cam\\"']
)

cc_library(
    name = "usb_cam",
    srcs = [
        "usb_cam.cc",
        "util.cc",
    ],
    hdrs = [
        "usb_cam.h",
        "util.h",
    ],
    
    deps = [
        "//modules/drivers/usb_cam/proto:usb_cam_proto",
        "//framework:cybertron",
        "@adv_plat//:adv_plat",
    ],
    copts = ['-mavx2']
)

cpplint()
