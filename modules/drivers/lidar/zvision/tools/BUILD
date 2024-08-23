load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "tools",
    srcs = [
        "tcp_client.cc",
        "tools.cc"
    ],
    hdrs = [
        "tcp_client.h",
        "tools.h"
    ],
    copts = ['-DMODULE_NAME=\\"zvision\\"'],
    deps = [
        "//cyber",
        "//modules/common/util:message_util",
        "//modules/drivers/lidar/common/driver_factory:apollo_lidar_driver_base",
        #"//modules/common/util",
        #"//modules/drivers/lidar/proto:config",
        "//modules/drivers/lidar/proto:config_cc_proto",
    ],
)

apollo_package()

cpplint()
