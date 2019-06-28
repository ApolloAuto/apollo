load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libcamera_component.so",
    linkopts = [
        "-lswscale",
        "-lavformat",
        "-lavcodec",
        "-latomic",
    ],
    linkshared = True,
    linkstatic = False,
    deps = [
        ":camera_component_lib",
        ":compress_component_lib",
    ],
)

cc_library(
    name = "camera_component_lib",
    srcs = ["camera_component.cc"],
    hdrs = ["camera_component.h"],
    copts = ['-DMODULE_NAME=\\"camera\\"'],
    deps = [
        ":camera",
        "//cyber",
        "//modules/drivers/proto:sensor_proto",
    ],
)

cc_library(
    name = "compress_component_lib",
    srcs = ["compress_component.cc"],
    hdrs = ["compress_component.h"],
    copts = ['-DMODULE_NAME=\\"camera\\"'],
    linkopts = [
        "-lopencv_core",
        "-lopencv_highgui",
    ],
    deps = [
        "//cyber",
        "//modules/drivers/camera/proto:camera_proto",
        "//modules/drivers/proto:sensor_proto",
    ],
)

cc_library(
    name = "camera",
    srcs = select({
        ":x86_mode": [
            "usb_cam.cc",
            "util.cc",
        ],
        ":arm_mode": [
            "usb_cam.cc",
        ],
    }),
    hdrs = select({
        ":x86_mode": [
            "usb_cam.h",
            "util.h",
        ],
        ":arm_mode": [
            "usb_cam.h",
        ],
    }),
    copts = select({
        ":x86_mode": ["-mavx2"],
        ":arm_mode": [""],
    }),
    deps = select({
        ":x86_mode": [
            "//cyber",
            "//modules/drivers/camera/proto:camera_proto",
            "@adv_plat",
        ],
        ":arm_mode": [
            "//cyber",
            "//modules/drivers/camera/proto:camera_proto",
        ],
    }),
)

config_setting(
    name = "x86_mode",
    values = {"cpu": "k8"},
)

config_setting(
    name = "arm_mode",
    values = {"cpu": "arm"},
)

cpplint()
