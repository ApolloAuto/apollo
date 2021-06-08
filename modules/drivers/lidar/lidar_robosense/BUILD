load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data = [
        ":runtime_data",
    ],
    targets = [
        "libsuteng_driver_component.so",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "conf/*.conf",
        "dag/*.dag",
        "launch/*.launch",
        "params/*.yaml",
    ]),
)

cc_binary(
    name = "libsuteng_driver_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":driver"],
)

cc_library(
    name = "driver",
    srcs = glob(
        ["**/*.cpp"],
    ),
    hdrs = glob(
        ["**/*.h"],
    ),
    copts = ['-DMODULE_NAME=\\"suteng\\"'],
    deps = [
        "//cyber",
        "//modules/common/util",
        "//modules/drivers/lidar/lidar_robosense/proto:suteng_proto",
        "//modules/drivers/proto:pointcloud_cc_proto",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@eigen",
        "@local_config_pcl//:pcl",
    ],
)

cpplint()
