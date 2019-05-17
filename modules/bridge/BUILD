load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "bridge_lib",
    srcs = [
    ],
    copts = ['-DMODULE_NAME=\\"bridge\\"'],
    deps = [
        "//cyber",
        "//external:gflags",
        "//modules/common",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/monitor_log",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/planning/proto:planning_proto",
        "//modules/bridge/common:bridge_gflags",
    ],
)

cc_library(
    name = "udp_bridge_component_lib",
    srcs = [
        "udp_bridge_component.cc",
    ],
    hdrs = [
        "udp_bridge_component.h",
    ],
    copts = ['-DMODULE_NAME=\\"bridge\\"'],
    deps = [
        ":bridge_lib",
    ],
)

cc_binary(
    name = "libudp_bridge_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":udp_bridge_component_lib"],
)

#cc_test(
#    name = "bridge_control_component_test",
#    size = "small",
#    srcs = [
#        "bridge_control_component_test.cc",
#    ],
#    data = ["//modules/control:control_testdata"],
#    deps = [
#        ":bridge_control_component_lib",
#        "@gtest//:main",
#    ],
#)

#filegroup(
#    name = "control_testdata",
#    srcs = glob([
#        "testdata/**",
#    ]),
#)

cpplint()
