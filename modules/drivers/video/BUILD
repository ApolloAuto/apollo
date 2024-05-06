load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_component", "apollo_cc_library")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_files",
    srcs = glob([
        "conf/*.txt",
        "dag/*.dag",
        "launch/*.launch",
    ])
)

apollo_component(
    name = "libvideo_driver_component.so",
    srcs = ["video_driver_component.cc"],
    hdrs = ["video_driver_component.h"],
    copts = ['-DMODULE_NAME=\\"video\\"'],
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/drivers/video:apollo_video_driver",
        "//modules/drivers/video:apollo_video_socket",
    ],
)

apollo_cc_library(
    name = "apollo_video_driver",
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

apollo_cc_library(
    name = "apollo_video_socket",
    srcs = ["socket_input.cc"],
    hdrs = [
        "input.h",
        "socket_input.h",
    ],
    deps = [
        "//cyber",
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "//modules/common_msgs/basic_msgs:header_cc_proto",
        "//modules/common/util:common_util",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
    ],
)

apollo_package()
cpplint()
