load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libcidiv2x_component.so",
    deps = [":cidiv2x_component_lib"],
    linkopts = ["-shared"],
    linkstatic = False,
)
cc_library(
    name = "cidiv2x_component_lib",
    srcs = ["cidiv2x_component.cc"],
    hdrs = ["cidiv2x_component.h"],
    deps = [
        "//cyber",
        "//modules/common/monitor_log",
        "//modules/drivers/cidiv2x/stream:cidiv2x_stream",
    ],
    copts = ['-DMODULE_NAME=\\"cidiv2x\\"']
)

#cc_library(
#    name = "cidiv2x_lib",
#    srcs = [
#        "cidiv2x.cc",
#    ],
#    hdrs = [
#        "cidiv2x.h",
#    ],
#    deps = [
#        "//modules/drivers/cidiv2x/stream:cidiv2x_stream",
#        "//modules/common",
#        "//modules/common:apollo_app",
#        "//modules/common/adapters:adapter_manager",
#        "//modules/common/monitor_log",
#        ":cidiv2x_gflags",
#    ],
#)
#
#cc_binary(
#    name = "cidiv2x",
#    srcs = ["main.cc"],
#    deps = [
#        ":cidiv2x_lib",
#        "//external:gflags",
#        "//modules/common:log",
#        "//modules/common/monitor_log",
#        "@ros//:ros_common",
#    ],
#)
#
#cc_library(
#    name = "cidiv2x_gflags",
#    srcs = [
#        "cidiv2x_gflags.cc",
#    ],
#    hdrs = [
#        "cidiv2x_gflags.h",
#    ],
#    deps = [
#        "//external:gflags",
#    ],
#)


cpplint()
