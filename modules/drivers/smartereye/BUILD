load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CAMERA_COPTS = ['-DMODULE_NAME=\\"camera\\"']

install(
    name = "install",
    data_dest = "drivers/addition_data/smartereye",
    library_dest = "drivers/lib/smartereye",
    data = [
        ":runtime_data",
    ],
    targets = [
        ":libsmartereye_component.so",
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

cc_binary(
    name = "libsmartereye_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [
        ":compress_component_lib",
        ":smartereye_component_lib",
    ],
)

cc_library(
    name = "smartereye_component_lib",
    srcs = ["smartereye_component.cc"],
    hdrs = ["smartereye_component.h"],
    alwayslink = True,
    copts = CAMERA_COPTS,
    deps = [
        ":smartereye_device",
        ":smartereye_handler",
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "//modules/common_msgs/basic_msgs:header_cc_proto",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
        "//modules/common_msgs/sensor_msgs:smartereye_cc_proto",
        "//third_party/camera_library/smartereye",
    ],
)

cc_library(
    name = "smartereye_device",
    srcs = ["smartereye_device.cc"],
    hdrs = ["smartereye_device.h"],
    copts = CAMERA_COPTS,
    deps = [
        ":smartereye_handler",
        "//cyber",
        "//modules/drivers/smartereye/proto:config_cc_proto",
        "//third_party/camera_library/smartereye",
    ],
)

cc_library(
    name = "smartereye_handler",
    srcs = ["smartereye_handler.cc"],
    hdrs = ["smartereye_handler.h"],
    copts = CAMERA_COPTS,
    deps = [
        "//cyber",
        "//third_party/camera_library/smartereye",
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
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
        "//modules/common_msgs/sensor_msgs:smartereye_cc_proto",
        "//modules/drivers/smartereye/proto:config_cc_proto",
        "@opencv//:highgui",
    ],
)

cpplint()
