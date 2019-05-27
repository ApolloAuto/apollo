load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "udp_bridge",
    copts = ['-DMODULE_NAME=\\"bridge\\"'],
    deps = [
        "//cyber",
        "//external:gflags",
<<<<<<< HEAD

=======
>>>>>>> bridge: add bridge receiver
        "//modules/bridge/common:bridge_buffer",
        "//modules/bridge/common:bridge_gflags",
        "//modules/bridge/common:macro",
        "//modules/bridge/common:util",
        "//modules/bridge/proto:udp_bridge_remote_info_proto",
        "//modules/common",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/monitor_log",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/planning/proto:planning_proto",
    ],
)

cc_library(
    name = "udp_bridge_sender",
    srcs = [
        "udp_bridge_sender_component.cc",
    ],
    hdrs = [
        "udp_bridge_sender_component.h",
    ],
    copts = ['-DMODULE_NAME=\\"bridge\\"'],
    deps = [
        ":udp_bridge",
    ],
)

cc_library(
    name = "udp_bridge_receiver",
    srcs = [
        "udp_bridge_receiver_component.cc",
    ],
    hdrs = [
        "udp_bridge_receiver_component.h",
    ],
    copts = ['-DMODULE_NAME=\\"bridge\\"'],
    deps = [
        ":udp_bridge",
    ],
)

cc_binary(
    name = "libudp_bridge_sender_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":udp_bridge_sender"],
)

cc_binary(
    name = "libudp_bridge_receiver_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":udp_bridge_receiver"],
)

cc_test(
    name = "udp_bridge_component_test",
    size = "small",
    srcs = [
        "udp_bridge_component_test.cc",
    ],
    deps = [
        ":udp_bridge_sender",
        "@gtest//:main",
    ],
)

cpplint()
