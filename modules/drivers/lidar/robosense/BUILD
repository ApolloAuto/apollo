load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library", "apollo_component")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "conf/*.conf",
        "dag/*.dag",
        "launch/*.launch",
        "params/**",
    ]),
)

apollo_component(
    name = "libsuteng_driver_component.so",
    srcs = glob(
        ["**/*.cpp"],
    ),
    hdrs = glob(
        ["**/*.h"],
    ),
    copts = ['-DMODULE_NAME=\\"suteng\\"'],
    deps = [
        "//cyber",
        "//modules/common/util:common_util",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/drivers/lidar/robosense/proto:sensor_suteng_cc_proto",
        "//modules/drivers/lidar/robosense/proto:lidars_filter_conf_proto",
        "//modules/drivers/lidar/robosense/proto:sensor_suteng_conf_proto",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@eigen",
        "@local_config_pcl//:pcl",
    ],
)

apollo_package()
cpplint()
