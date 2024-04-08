load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
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

apollo_cc_library(
    name = "apollo_bridge_common",
    srcs = [
        "common/bridge_buffer.cc",
        "common/bridge_gflags.cc",
        "common/bridge_header.cc",
        "common/util.cc",
    ],
    hdrs = [
        "common/bridge_buffer.h",
        "common/bridge_gflags.h",
        "common/bridge_header.h",
        "common/bridge_header_item.h",
        "common/bridge_proto_diser_buf_factory.h",
        "common/bridge_proto_diserialized_buf.h",
        "common/bridge_proto_serialized_buf.h",
        "common/macro.h",
        "common/udp_listener.h",
        "common/util.h",
    ],
    deps = [
        "//modules/common/adapters:adapter_gflags",
        "@com_github_gflags_gflags//:gflags",
    ],
)

apollo_cc_test(
    name = "bridge_buffer_test",
    size = "small",
    srcs = ["common/bridge_buffer_test.cc"],
    deps = [
        ":apollo_bridge_common",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "bridge_proto_buf_test",
    size = "small",
    srcs = ["common/bridge_proto_buf_test.cc"],
    deps = [
        "//modules/bridge:apollo_udp_bridge",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_library(
    name = "apollo_udp_bridge",
    copts = BRIDGE_COPTS,
    deps = [
        ":apollo_bridge_common",
        "//cyber",
        "//modules/bridge/proto:udp_bridge_remote_info_cc_proto",
        "//modules/common/monitor_log",
        "//modules/common/util:common_util",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
    ],
)

apollo_component(
    name = "libudp_bridge_sender_component.so",
    srcs = [
        "udp_bridge_sender_component.cc",
    ],
    hdrs = [
        "udp_bridge_sender_component.h",
    ],
    deps = [":apollo_udp_bridge"],
)

apollo_component(
    name = "libudp_bridge_receiver_component.so",
    srcs = [
        "udp_bridge_receiver_component.cc",
    ],
    hdrs = [
        "udp_bridge_receiver_component.h",
    ],
    deps = [":apollo_udp_bridge"],
)

apollo_component(
    name = "libudp_bridge_multi_receiver_component.so",
    srcs = [
        "udp_bridge_multi_receiver_component.cc",
    ],
    hdrs = [
        "udp_bridge_multi_receiver_component.h",
    ],
    deps = [":apollo_udp_bridge"],
)

apollo_cc_test(
    name = "udp_bridge_component_test",
    size = "small",
    srcs = ["udp_bridge_component_test.cc"],
    linkstatic = True,
    deps = [
        ":apollo_udp_bridge",
        "@com_google_googletest//:gtest_main",
        # internal component library if you really need these component depends
        ":DO_NOT_IMPORT_udp_bridge_multi_receiver_component",
        ":DO_NOT_IMPORT_udp_bridge_receiver_component",
        ":DO_NOT_IMPORT_udp_bridge_sender_component",
    ],
)

apollo_package()

cpplint()
