load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "conf/*.conf",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

apollo_component(
    name = "liblidar_driver_component.so",
    srcs = ["lidar_driver_component.cc"],
    hdrs = ["lidar_driver_component.h"],
    deps = [
        "//cyber",
        "//modules/drivers/lidar/common/driver_factory:lidar_driver_factory",
        "//modules/drivers/lidar/proto:config_cc_proto",
    ],
)

apollo_package()

cpplint()
