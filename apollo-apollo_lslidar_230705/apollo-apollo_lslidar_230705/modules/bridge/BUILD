load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

BRIDGE_COPTS = ['-DMODULE_NAME=\\"bridge\\"']

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

install(
    name = "install",
    library_dest = "bridge/lib",
    data_dest = "bridge",
    data = [
      ":runtime_data",
      "cyberfile.xml",
      "bridge.BUILD"
    ],
    targets = [
      ":libudp_bridge_sender_component.so",
      ":libudp_bridge_receiver_component.so",
      ":libudp_bridge_multi_receiver_component.so",
    ],
    deps = [
        "//modules/bridge/test:install",
        "//modules/bridge/proto:py_pb_bridge",
        "//modules/bridge/common:install",
    ],
)
install(
    name = "pb_hdrs",
    data_dest = "bridge/include",
    data = [
        "//modules/bridge/proto:udp_bridge_remote_info_cc_proto",
    ],
)

cc_library(
    name = "udp_bridge",
    copts = BRIDGE_COPTS,
    deps = [
        "//cyber",
        "//modules/bridge/common:bridge_gflags",
        "//modules/bridge/common:bridge_header",
        "//modules/bridge/common:bridge_proto_diser_buf_factory",
        "//modules/bridge/common:bridge_proto_diserialized_buf",
        "//modules/bridge/common:bridge_proto_serialized_buf",
        "//modules/bridge/common:macro",
        "//modules/bridge/common:util",
        "//modules/bridge/proto:udp_bridge_remote_info_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/common/monitor_log",
        "//modules/common/util",
    ],
)

cc_library(
    name = "udp_bridge_sender",
    srcs = ["udp_bridge_sender_component.cc"],
    hdrs = ["udp_bridge_sender_component.h"],
    copts = BRIDGE_COPTS,
    alwayslink = True,
    deps = [
        ":udp_bridge",
    ],
)

cc_library(
    name = "udp_bridge_receiver",
    srcs = ["udp_bridge_receiver_component.cc"],
    hdrs = ["udp_bridge_receiver_component.h"],
    copts = BRIDGE_COPTS,
    alwayslink = True,
    deps = [
        ":udp_bridge",
        "//modules/bridge/common:udp_listener",
    ],
)

cc_library(
    name = "udp_bridge_multi_receiver",
    srcs = ["udp_bridge_multi_receiver_component.cc"],
    hdrs = ["udp_bridge_multi_receiver_component.h"],
    copts = BRIDGE_COPTS,
    alwayslink = True,
    deps = [
        ":udp_bridge",
        "//modules/bridge/common:udp_listener",
    ],
)

cc_binary(
    name = "libudp_bridge_sender_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":udp_bridge_sender"],
)

cc_binary(
    name = "libudp_bridge_receiver_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":udp_bridge_receiver"],
)

cc_binary(
    name = "libudp_bridge_multi_receiver_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":udp_bridge_multi_receiver"],
)

cc_test(
    name = "udp_bridge_component_test",
    size = "small",
    srcs = ["udp_bridge_component_test.cc"],
    deps = [
        ":udp_bridge_sender",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)
install_src_files(
    name = "install_src",
    deps = [
        ":install_bridge_src",
        ":install_bridge_hdrs"
    ],
)

install_src_files(
    name = "install_bridge_src",
    src_dir = ["."],
    dest = "bridge/src",
    filter = "*",
)

install_src_files(
    name = "install_bridge_hdrs",
    src_dir = ["."],
    dest = "bridge/include",
    filter = "*.h",
)

cpplint()
