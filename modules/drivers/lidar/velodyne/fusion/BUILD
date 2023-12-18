load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library", "apollo_component")

package(default_visibility = ["//visibility:public"])

apollo_component(
    name = "libvelodyne_fusion_component.so",
    srcs = ["pri_sec_fusion_component.cc"],
    hdrs = ["pri_sec_fusion_component.h"],
    copts = ['-DMODULE_NAME=\\"velodyne\\"'],
    deps = [
        "//cyber",
        "//modules/drivers/lidar/proto:velodyne_config_cc_proto",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/transform:apollo_transform",
        "@eigen",
    ],
)

apollo_package()
cpplint()
