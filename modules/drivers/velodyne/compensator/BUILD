load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libvelodyne_compensator_component.so",
    deps = [":compensator_component_lib"],
    linkopts = ["-shared"],
    linkstatic = False,
)

cc_library(
    name = "compensator_component_lib",
    srcs = ["compensator_component.cc"],
    hdrs = ["compensator_component.h"],
    deps = [
        "//modules/drivers/velodyne/compensator:compensator_lib",
        "//framework:cybertron",
    ],
    copts = ['-DMODULE_NAME=\\"velodyne\\"']
)

cc_library(
    name = "compensator_lib",
    srcs = [
        "compensator.cc",
    ],
    hdrs = [
        "compensator.h",
    ],
    deps = [
        "//modules/drivers/velodyne/proto:velodyne_proto",
        "//modules/drivers/proto:sensor_proto",
        "//modules/transform:tf2_buffer_lib",
        "//framework:cybertron",
        "@eigen",
    ],
    copts = ['-DMODULE_NAME=\\"velodyne\\"']
)

cpplint()
