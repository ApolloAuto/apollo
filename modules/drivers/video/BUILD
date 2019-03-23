load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libvideo_driver_component.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":video_driver_component_lib"],
)

cc_library(
    name = "video_driver_component_lib",
    srcs = ["video_driver_component.cc"],
    hdrs = ["video_driver_component.h"],
    copts = ['-DMODULE_NAME=\\"video\\"'],
    deps = [
        "//cyber",
        "//modules/common/util:message_util",
        "//modules/drivers/video:video_lib",
    ],
)

cc_library(
    name = "video_lib",
    srcs = [
        "driver.cc",
        "socket_input.cc",
    ],
    hdrs = [
        "driver.h",
        "input.h",
        "socket_input.h",
    ],
    deps = [
        "//cyber",
        "//modules/common/util",
        "//modules/drivers/video/proto:video_proto",
        "//modules/drivers/proto:sensor_proto",
    ],
)

cpplint()
