load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

HESAI_COPTS = ['-DMODULE_NAME=\\"hesai\\"']

cc_library(
    name = "hesai_component",
    srcs = ["hesai_component.cc"],
    hdrs = ["hesai_component.h"],
    copts = HESAI_COPTS,
    deps = [
        ":driver",
        "//cyber",
        "//modules/drivers/lidar/hesai:parser_factory",
        "//modules/drivers/lidar/proto:hesai_config_cc_proto",
    ],
    alwayslink = True,
)

cc_library(
    name = "driver",
    srcs = ["driver.cc"],
    hdrs = ["driver.h"],
    copts = HESAI_COPTS,
    deps = [
        "//cyber",
        "//modules/drivers/lidar/common/driver_factory:driver_base",
        "//modules/drivers/lidar/hesai:parser",
        "//modules/drivers/lidar/hesai:parser_factory",
        "//modules/drivers/lidar/hesai:type_defs",
        "//modules/drivers/lidar/hesai:udp_input",
        "//modules/drivers/lidar/proto:config_cc_proto",
    ],
)

cpplint()
