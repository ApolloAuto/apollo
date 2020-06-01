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
    srcs = select(
        {
            "//tools/platform:x86_mode": [
                "usb_cam.cc",
                "util.cc",
            ],
            "//tools/platform:aarch64_mode": [
                "usb_cam.cc",
            ],
        },
        no_match_error = "Please Build with an ARM or Linux x86_64 platform",
    ),
    hdrs = select(
        {
            "//tools/platform:x86_mode": [
                "usb_cam.h",
                "util.h",
            ],
            "//tools/platform:aarch64_mode": [
                "usb_cam.h",
            ],
        },
        no_match_error = "Please Build with an ARM or Linux x86_64 platform",
    ),
    copts = select({
        "//tools/platform:x86_mode": ["-mavx2"],
        "//tools/platform:aarch64_mode": [""],
    }),
    deps = select(
        {
            "//tools/platform:x86_mode": [
                "//cyber",
                "//modules/drivers/camera/proto:camera_proto",
                "@adv_plat",
            ],
            "//tools/platform:aarch64_mode": [
                "//cyber",
                "//modules/drivers/camera/proto:camera_proto",
            ],
        },
        no_match_error = "Please Build with an ARM or Linux x86_64 platform",
    ),
)

cpplint()
