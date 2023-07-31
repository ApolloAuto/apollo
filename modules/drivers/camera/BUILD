load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
# load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_x86_64", "if_aarch64", "if_gpu")

package(default_visibility = ["//visibility:public"])

CAMERA_COPTS = ['-DMODULE_NAME=\\"camera\\"']

# install_src_files(
#     name = "install_src",
#     deps = [
#         ":install_drivers_camera_src",
#         ":install_drivers_camera_hdrs"
#     ],
# )

# install_src_files(
#     name = "install_drivers_camera_src",
#     src_dir = ["."],
#     dest = "drivers-camera/src",
#     filter = "*",
# )

# install_src_files(
#     name = "install_drivers_camera_hdrs",
#     src_dir = ["."],
#     dest = "drivers-camera/include",
#     filter = "*.h",
# )

# install(
#     name = "pb_hdrs",
#     data_dest = "drivers-camera/include",
#     data = [
#         "//modules/drivers/camera/proto:config_cc_proto",
#     ],
# )

# install_files(
#     name = "pb_drivers_camera_py",
#     dest = "drivers-camera/python/modules/drivers/camera",
#     files = [
#         "//modules/drivers/camera/proto:config_py_pb2",
#     ],
# )

cc_binary(
    name = "libcamera_component.so",
    linkopts = [
        "-latomic",
    ],
    linkshared = True,
    linkstatic = True,
    deps = [
        ":camera_component_lib",
        "@ffmpeg//:avcodec",
        "@ffmpeg//:avformat",
        "@ffmpeg//:swscale",
    ],
)

cc_binary(
    name = "libcamera_compress_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [
        ":compress_component_lib",
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

# filegroup(
#     name = "package_data",
#     srcs = [
#         ":cyberfile.xml",
#         ":drivers-camera.BUILD",
#     ],
# )

# install(
#     name = "install_package_data",
#     data_dest = "drivers-camera",
#     data = ["package_data"],
# )

# install(
#     name = "install",
#     data_dest = "drivers-camera/addition_data",
#     library_dest = "drivers-camera/lib",
#     data = [
#       ":runtime_data",
#     ],
#     targets = [
#       ":libcamera_component.so",
#       ":libcamera_compress_component.so"
#     ],
#     deps = [
#         ":pb_hdrs", 
#         ":pb_drivers_camera_py",
#         "install_package_data"
#     ]
# )

cc_library(
    name = "camera_component_lib",
    srcs = ["camera_component.cc"],
    hdrs = ["camera_component.h"],
    alwayslink = True,
    copts = CAMERA_COPTS,
    deps = [
        ":camera",
        "//cyber",
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "//modules/common_msgs/basic_msgs:header_cc_proto",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
    ],
)

cc_library(
    name = "compress_component_lib",
    srcs = ["compress_component.cc"],
    hdrs = ["compress_component.h"],
    copts = CAMERA_COPTS,
    alwayslink = True,
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

cc_library(
    name = "camera",
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

apollo_package()
cpplint()
