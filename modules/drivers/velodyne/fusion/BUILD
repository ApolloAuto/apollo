load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libvelodyne_fusion_component.so",
    deps = [":fusion_component_lib"],
    linkshared = True,
    linkstatic = False,
)

cc_library(
    name = "fusion_component_lib",
    srcs = ["pri_sec_fusion_component.cc"],
    hdrs = ["pri_sec_fusion_component.h"],
    deps = [
        "//cyber",
        "//modules/drivers/velodyne/proto:velodyne_proto",
        "//modules/drivers/proto:sensor_proto",
        "//modules/transform:tf2_buffer_lib",
        "@eigen",
    ],
    copts = ['-DMODULE_NAME=\\"velodyne\\"']
)

cpplint()
