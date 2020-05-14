load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libsmartereye_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [
        ":smartereye_component_lib",
        ":compress_component_lib",
    ],
)

cc_library(
    name = "smartereye_component_lib",
    srcs = ["smartereye_component.cc"],
    hdrs = ["smartereye_component.h"],
    copts = ['-DMODULE_NAME=\\"camera\\"'],
    deps = [
        ":smartereye_device",
        ":smartereye_handler",
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/drivers/proto:sensor_proto",
        "//third_party/camera_library/smartereye:smartereye",
    ],
)

cc_library(
    name = "smartereye_device",
    srcs = ["smartereye_device.cc"],
    hdrs = ["smartereye_device.h"],
    copts = ['-DMODULE_NAME=\\"camera\\"'],
    deps = [
        ":smartereye_handler",
        "//cyber",
        "//modules/drivers/proto:sensor_proto",
        "//modules/drivers/smartereye/proto:camera_proto",
        "//third_party/camera_library/smartereye:smartereye",
    ],
)

cc_library(
    name = "smartereye_handler",
    srcs = ["smartereye_handler.cc"],
    hdrs = ["smartereye_handler.h"],
    copts = ['-DMODULE_NAME=\\"camera\\"'],
    deps = [
        "//cyber",
        "//modules/drivers/proto:sensor_proto",
        "//modules/drivers/smartereye/proto:camera_proto",
        "//third_party/camera_library/smartereye:smartereye",
    ],
)

cc_library(
    name = "compress_component_lib",
    srcs = ["compress_component.cc"],
    hdrs = ["compress_component.h"],
    copts = ['-DMODULE_NAME=\\"camera\\"'],
    linkopts = [
        "-lopencv_core",
        "-lopencv_highgui",
    ],
    deps = [
        "//cyber",
        "//modules/drivers/smartereye/proto:camera_proto",
        "//modules/drivers/proto:sensor_proto",
    ],
)

cpplint()
