load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libcamera_component.so",
    deps = [":usb_cam_component_lib"],
    linkopts = [
       "-shared",
       "-lswscale",
       "-lavformat",
       "-lavcodec",
    ],
    linkshared = True,
    linkstatic = False,
)

cc_library(
    name = "usb_cam_component_lib",
    srcs = ["usb_cam_component.cc"],
    hdrs = ["usb_cam_component.h"],
    deps = [
        ":camera",
        "//modules/drivers/proto:sensor_proto",
        "//framework:cybertron",
    ],
    copts = ['-DMODULE_NAME=\\"camera\\"']
)

cc_library(
    name = "camera",
    srcs = [
        "usb_cam.cc",
        "util.cc",
    ],
    hdrs = [
        "usb_cam.h",
        "util.h",
    ],
    
    deps = [
        "//modules/drivers/camera/proto:camera_proto",
        "//framework:cybertron",
        "@adv_plat//:adv_plat",
    ],
    copts = ['-mavx2']
)

cpplint()
