load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library", "apollo_component")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

LIVOX_COPTS = ['-DMODULE_NAME=\\"livox\\"']

apollo_component(
    name = "liblivox_component.so",
    copts = LIVOX_COPTS,
    srcs = [
        "component/livox_component.cpp",
        "component/livox_dispatcher.cpp",
        "driver/livox_util.cpp",
    ],
    hdrs = [
        "component/livox_component.h",
        "component/livox_obserable_binder.h",
        "component/livox_dispatcher.h",
        "driver/livox_util.h",
    ],
    deps = [
        "//modules/drivers/lidar/livox/proto:livox_proto",
        "//modules/drivers/lidar/common:lidar_common",
    ],
    linkopts = [
        "-llivox_lidar_sdk_shared",
    ]
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "conf/*.conf",
        "conf/*.json",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

apollo_package()
cpplint()
