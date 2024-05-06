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
    name = "libcompensator_component.so",
    srcs = ["compensator_component.cc", "compensator.cc"],
    hdrs = ["compensator_component.h", "compensator.h"],
    copts = ['-DMODULE_NAME=\\"compensator\\"'],
    deps = [
        "//cyber",
         "@eigen",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/latency_recorder",
        "//modules/drivers/lidar/compensator/proto:compensator_config_proto",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/transform:apollo_transform",
    ],
)

apollo_package()
cpplint()
