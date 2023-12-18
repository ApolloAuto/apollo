load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

HESAI_COPTS = ['-DMODULE_NAME=\\"hesai\\"']

apollo_cc_library(
    name = "hesai_component",
    srcs = ["hesai_component.cc"],
    hdrs = ["hesai_component.h"],
    copts = HESAI_COPTS,
    deps = [
        ":apollo_hesai_driver",
        "//cyber",
        "//modules/drivers/lidar/hesai:hesai_convert_component",
        "//modules/drivers/lidar/proto:hesai_config_cc_proto",
    ],
)

apollo_cc_library(
    name = "apollo_hesai_driver",
    srcs = ["driver.cc"],
    hdrs = ["driver.h"],
    copts = HESAI_COPTS,
    deps = [
        "//cyber",
        "//modules/drivers/lidar/common/driver_factory:apollo_lidar_driver_base",
        "//modules/drivers/lidar/hesai:hesai_convert_component",
        "//modules/drivers/lidar/proto:config_cc_proto",
    ],
)

apollo_package()
cpplint()