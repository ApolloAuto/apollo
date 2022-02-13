load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_x86_64")

package(default_visibility = ["//visibility:public"])

CAMERA_COPTS = ['-DMODULE_NAME=\\"wato\\"']

cc_binary(
    name = "libwato_component.so",
    linkopts = [
        "-latomic",
    ],
    linkshared = True,
    linkstatic = False,
    deps = [
        ":wato_component_lib",
        ":compress_component_lib",
        "@ffmpeg//:avcodec",
        "@ffmpeg//:avformat",
        "@ffmpeg//:swscale",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

install(
    name = "install",
    data = [
      ":runtime_data",
    ],
    targets = [
      ":libwato_component.so",
    ],
    deps = [
        "//cyber:install",
    ],
)

cc_library(
    name = "wato_component_lib",
    srcs = ["wato_component.cc"],
    hdrs = ["wato_component.h"],
    copts = CAMERA_COPTS,
    deps = [
        ":wato",
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
    copts = CAMERA_COPTS,
    deps = [
        "//cyber",
        "//modules/common/proto:error_code_cc_proto",
        "//modules/common/proto:header_cc_proto",
        "//modules/drivers/wato/proto:config_cc_proto",
        "//modules/drivers/proto:sensor_image_cc_proto",
        "@opencv//:core",
        "@opencv//:highgui",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc",
    ],
)

cc_library(
    name = "wato",
    srcs = select(
        {
            "@platforms//cpu:x86_64": [
                "usb_cam.cc",
                "util.cc",
            ],
            "@platforms//cpu:aarch64": [
                "usb_cam.cc",
            ],
        },
        no_match_error = "Please Build with an ARM or Linux x86_64 platform",
    ),
    hdrs = select(
        {
            "@platforms//cpu:x86_64": [
                "usb_cam.h",
                "util.h",
            ],
            "@platforms//cpu:aarch64": [
                "usb_cam.h",
            ],
        },
        no_match_error = "Please Build with an ARM or Linux x86_64 platform",
    ),
    deps = [
        "//cyber",
        "//modules/drivers/wato/proto:config_cc_proto",
        "@ffmpeg//:avcodec",
    ] + if_x86_64(
        ["@adv_plat"],
    ),
)

cpplint()
