load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libcamera_component.so",
    deps = [
        ":camera_component_lib",
        ":compress_component_lib",
    ],
    linkopts = [
       "-lswscale",
       "-lavformat",
       "-lavcodec",
       "-latomic",
    ],
    linkshared = True,
    linkstatic = False,
)

cc_library(
    name = "camera_component_lib",
    srcs = ["camera_component.cc"],
    hdrs = ["camera_component.h"],
    deps = [
        ":camera",
        "//cyber",
        "//modules/drivers/proto:sensor_proto",
    ],
    copts = ['-DMODULE_NAME=\\"camera\\"']
)

cc_library(
    name = "compress_component_lib",
    srcs = ["compress_component.cc"],
    hdrs = ["compress_component.h"],
    deps = [
        "//cyber",
        "//modules/drivers/camera/proto:camera_proto",
        "//modules/drivers/proto:sensor_proto",
        "@opencv2//:core",
        "@opencv2//:highgui"
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
        "//cyber",
        "//modules/drivers/camera/proto:camera_proto",
        "@adv_plat//:adv_plat",
    ],
    copts = ['-mavx2']
)

cpplint()
