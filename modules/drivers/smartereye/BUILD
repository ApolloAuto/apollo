load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_component", "apollo_cc_library")

package(default_visibility = ["//visibility:public"])

CAMERA_COPTS = ['-DMODULE_NAME=\\"camera\\"']


filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

apollo_component(
    name = "libsmartereye_component.so",
    srcs = [
        "compress_component.cc",
        "smartereye_component.cc",
    ],
    hdrs = [
        "compress_component.h",
        "smartereye_component.h",
    ],
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
        "//modules/drivers/smartereye/proto:config_cc_proto",
        "@opencv//:highgui",
    ],
)

apollo_cc_library(
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

apollo_cc_library(
    name = "smartereye_handler",
    srcs = ["smartereye_handler.cc"],
    hdrs = ["smartereye_handler.h"],
    copts = CAMERA_COPTS,
    deps = [
        "//cyber",
        "//third_party/camera_library/smartereye",
    ],
)

apollo_package()
cpplint()
