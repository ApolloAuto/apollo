load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "libvelodyne_compensator_component.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":compensator_component_lib"],
)

cc_library(
    name = "compensator_component_lib",
    srcs = ["compensator_component.cc"],
    hdrs = ["compensator_component.h"],
    copts = ['-DMODULE_NAME=\\"velodyne\\"'],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/latency_recorder",
        "//modules/common/time",
        "//modules/drivers/velodyne/compensator:compensator_lib",
        "//modules/drivers/velodyne/proto:velodyne_cc_proto",
    ],
)

cc_library(
    name = "compensator_lib",
    srcs = ["compensator.cc"],
    hdrs = ["compensator.h"],
    copts = ['-DMODULE_NAME=\\"velodyne\\"'],
    deps = [
        "//modules/drivers/proto:pointcloud_cc_proto",
        "//modules/drivers/velodyne/proto:config_cc_proto",
        "//modules/transform:tf2_buffer_lib",
        "@eigen",
    ],
)

cpplint()
