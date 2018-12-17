load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libvelodyne_fusion_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":fusion_component_lib"],
)

cc_library(
    name = "fusion_component_lib",
    srcs = ["pri_sec_fusion_component.cc"],
    hdrs = ["pri_sec_fusion_component.h"],
    copts = ['-DMODULE_NAME=\\"velodyne\\"'],
    deps = [
        "//cyber",
        "//modules/drivers/proto:sensor_proto",
        "//modules/drivers/velodyne/proto:velodyne_proto",
        "//modules/transform:tf2_buffer_lib",
        "@eigen",
    ],
)

cpplint()
