load("//tools:apollo_package.bzl", "apollo_package", "apollo_component")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_x86_64", "if_aarch64", "if_gpu")


package(default_visibility = ["//visibility:public"])

COPTS = ['-DMODULE_NAME=\\"resource_statistic\\"']

apollo_component(
    name = "libresource_statistic_component.so",
    srcs = [
        "resource_statistic_gflags.cc",
        "resource_statistic_component.cc",
        "cpu_resource_statistic.cc",
        "gpu_resource_statistic.cc",
        "disk_resource_statistic.cc",
        "memory_resource_statistic.cc",
        "util.cc"

    ],
    hdrs = [
        "resource_statistic_gflags.h",
        "resource_statistic_component.h",
        "cpu_resource_statistic.h",
        "gpu_resource_statistic.h",
        "disk_resource_statistic.h",
        "memory_resource_statistic.h",
        "util.h"
    ],
    copts = COPTS,
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/statistics_msgs:resource_statistic_cc_proto",
    ] + if_x86_64(
        ["@local_config_cuda//cuda:nvidia_ml"]
    )
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*",
        "dag/*",
    ]),
)

apollo_package(enable_source=False)
cpplint()
