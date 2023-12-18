load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_component", "apollo_package")
load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

# install(
#     name = "install",
#     library_dest = "drivers/lib/lidar/lslidar/driver",
#     targets = [
#         ":liblslidar_driver_component.so",
#     ],
# )

apollo_cc_binary(
    name = "liblslidar_driver_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":lslidar_driver_component_lib"],
)

apollo_cc_library(
    name = "lslidar_driver_component_lib",
    srcs = ["lslidar_driver_component.cc"],
    hdrs = ["lslidar_driver_component.h"],
    copts = ['-DMODULE_NAME=\\"lslidar\\"'],
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/drivers/lidar/lslidar/driver",
    ],
)

apollo_cc_library(
    name = "driver",
    srcs = [
        "driver.cc",
        "input.cc",
    ],
    hdrs = [
        "driver.h",
        "input.h",
    ],
    copts = ['-DMODULE_NAME=\\"lslidar\\"'],
    linkopts = [
        "-lpcap",
    ],
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/drivers/lidar/lslidar/proto:config_cc_proto",
    ],
)

apollo_package()

cpplint()
