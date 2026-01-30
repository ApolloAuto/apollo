load("//tools:apollo_package.bzl", "apollo_package", "apollo_component", "apollo_cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_x86_64", "if_aarch64", "if_gpu")

package(default_visibility = ["//visibility:public"])

CAMERA_COPTS = ['-DMODULE_NAME=\\"camera\\"']

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

apollo_component(
    name = "libcamera_component.so",
    srcs = ["camera_component.cc"],
    hdrs = ["camera_component.h"],
    linkopts = [
        "-latomic",
    ],
    copts = CAMERA_COPTS,
    deps = [
        ":apollo_camera_util",
        "//cyber",
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "//modules/common_msgs/basic_msgs:header_cc_proto",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
        "@ffmpeg//:avcodec",
        "@ffmpeg//:avformat",
        "@ffmpeg//:swscale",
    ],
)

apollo_component(
    name = "libcamera_compress_component.so",
    srcs = ["compress_component.cc"],
    hdrs = ["compress_component.h"],
    copts = CAMERA_COPTS,
    deps = [
        "//cyber",
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "//modules/common_msgs/basic_msgs:header_cc_proto",
        "//modules/drivers/camera/proto:config_cc_proto",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
    ] + if_aarch64(
        [
            "@nvjpeg//:nvjpeg"
        ], [
            "@opencv//:core",
            "@opencv//:highgui",
            "@opencv//:imgcodecs",
            "@opencv//:imgproc",
        ],
    ),
)

apollo_cc_library(
    name = "apollo_camera_util",
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
        "@boost",
        "//cyber",
        "//modules/drivers/camera/proto:config_cc_proto",
        "@ffmpeg//:avcodec",
    ] + if_x86_64(
        ["@adv_plat"],
    ) + if_aarch64(
        ["//modules/drivers/camera/format:convert",]
    ),
    copts = [ 
        '-fopenmp',
    ],
)

apollo_package(enable_source=False)
cpplint()
