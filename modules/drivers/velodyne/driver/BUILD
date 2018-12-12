load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libvelodyne_driver_component.so",
    deps = [":velodyne_driver_component_lib"],
    linkopts = ["-shared"],
    linkstatic = False,
)

cc_library(
    name = "velodyne_driver_component_lib",
    srcs = ["velodyne_driver_component.cc"],
    hdrs = ["velodyne_driver_component.h"],
    deps = [
        "//cyber",
        "//modules/common/util:message_util",
        "//modules/drivers/velodyne/driver:driver",
    ],
    copts = ['-DMODULE_NAME=\\"velodyne\\"']
)

cc_library(
    name = "driver",
    srcs = [
        "driver.cc",
        "driver64.cc",
        "input.cc",
        "socket_input.cc",
    ],
    hdrs = [
        "driver.h",
        "input.h",
        "socket_input.h",
    ],
    linkopts = [
        "-lpcap",
    ],
    deps = [
        "//cyber",
        "//modules/common/util",
        "//modules/drivers/velodyne/proto:velodyne_proto",
        "@pcl",
    ],
    copts = ['-DMODULE_NAME=\\"velodyne\\"']
)

cpplint()
