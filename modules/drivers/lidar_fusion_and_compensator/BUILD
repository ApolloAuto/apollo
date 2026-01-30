load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library", "apollo_component")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

apollo_component(
    name = "liblidar_fusion_and_compensator_component.so",
    srcs = ["fusion_and_compensator_component.cc"],
    hdrs = ["fusion_and_compensator_component.h"],
    copts = ['-DMODULE_NAME=\\"fusion_and_compensator\\"'],
    deps = [
        "//cyber",
        "//modules/drivers/lidar_fusion_and_compensator/proto:config_cc_proto",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/transform:apollo_transform",
        "@eigen",
    ],
)

apollo_package(enable_source=False)
cpplint()
