load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data_dest = "drivers/addition_data/video",
    library_dest = "drivers/lib/video",
    data = [
        ":runtime_files",
    ],
    targets = [
        "libvideo_driver_component.so",
    ],
)

filegroup(
    name = "runtime_files",
    srcs = glob([
        "conf/*.txt",
        "dag/*.dag",
        "launch/*.launch",
    ])
)

cc_binary(
    name = "libvideo_driver_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":video_driver_component_lib"],
)

cc_library(
    name = "video_driver_component_lib",
    srcs = ["video_driver_component.cc"],
    hdrs = ["video_driver_component.h"],
    copts = ['-DMODULE_NAME=\\"video\\"'],
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/drivers/video:driver",
        "//modules/drivers/video:socket",
    ],
)

cc_library(
    name = "driver",
    srcs = ["driver.cc"],
    hdrs = [
        "driver.h",
        "socket_input.h",
    ],
    deps = [
        "//cyber",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
        "//modules/drivers/video/proto:video_h265cfg_cc_proto",
    ],
)

cc_library(
    name = "socket",
    srcs = ["socket_input.cc"],
    hdrs = [
        "input.h",
        "socket_input.h",
    ],
    deps = [
        "//cyber",
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "//modules/common_msgs/basic_msgs:header_cc_proto",
        "//modules/common/util",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
    ],
)

cpplint()
