load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library", "apollo_component")

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
    name = "libfusion_component.so",
    srcs = ["pri_sec_fusion_component.cc"],
    hdrs = ["pri_sec_fusion_component.h"],
    copts = ['-DMODULE_NAME=\\"fusion\\"'],
    deps = [
        "//cyber",
        "//modules/drivers/lidar/fusion/proto:fusion_config_proto",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/transform:apollo_transform",
        "@eigen",
    ],
)

apollo_package()
cpplint()
