load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
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
        "//modules/common/proto:error_code_cc_proto",
        "//modules/common/proto:header_cc_proto",
        "//modules/drivers/proto:sensor_image_cc_proto",
    ],
)

cc_library(
    name = "compress_component_lib",
    srcs = ["compress_component.cc"],
    hdrs = ["compress_component.h"],
    copts = ['-DMODULE_NAME=\\"camera\\"'],
    deps = [
        "//cyber",
        "//modules/common/proto:error_code_cc_proto",
        "//modules/common/proto:header_cc_proto",
        "//modules/drivers/camera/proto:config_cc_proto",
        "//modules/drivers/proto:sensor_image_cc_proto",
        "@opencv",
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
                "//modules/drivers/camera/proto:config_cc_proto",
                "@adv_plat",
            ],
            "//tools/platform:aarch64_mode": [
                "//cyber",
                "//modules/drivers/camera/proto:config_cc_proto",
            ],
        },
        no_match_error = "Please Build with an ARM or Linux x86_64 platform",
    ),
)

cpplint()
