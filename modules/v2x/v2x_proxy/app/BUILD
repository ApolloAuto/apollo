load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

V2X_COPTS = ["-DMODULE_NAME=\\\"v2x\\\""]

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "utils",
    srcs = ["utils.cc"],
    hdrs = ["utils.h"],
    copts = V2X_COPTS,
    deps = [
        "//cyber",
        "//modules/common/math:quaternion",
        "//modules/map/hdmap:hdmap_util",
        "//modules/v2x/v2x_proxy/obu_interface:obu_interface_grpc_impl",
        "//modules/v2x/v2x_proxy/os_interface",
        "//modules/v2x/v2x_proxy/proto_adapter",
    ],
)

cc_library(
    name = "v2x_proxy",
    srcs = ["v2x_proxy.cc"],
    hdrs = ["v2x_proxy.h"],
    copts = V2X_COPTS,
    deps = [
        ":proxy",
        ":utils",
        "//cyber",
        "//modules/map/hdmap:hdmap_util",
        "//modules/v2x/v2x_proxy/obu_interface:obu_interface_grpc_impl",
        "//modules/v2x/v2x_proxy/os_interface",
        "//modules/v2x/v2x_proxy/proto_adapter",
    ],
)

cc_library(
    name = "hik_proxy",
    srcs = ["hik_proxy.cc"],
    hdrs = ["hik_proxy.h"],
    copts = V2X_COPTS,
    deps = [
        ":proxy",
        ":utils",
        "//cyber",
        "//modules/bridge/common:udp_listener",
        "//modules/canbus/proto:chassis_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/util:message_util",
        "//modules/localization/proto:localization_cc_proto",
        "//modules/map/hdmap:hdmap_util",
        "//modules/perception/proto:traffic_light_detection_cc_proto",
        "//modules/v2x/common:json_parse",
        "//modules/v2x/common:v2x_proxy_gflags",
        "//modules/v2x/proto:hik_message_cc_proto",
        "//modules/v2x/proto:spat_data_cc_proto",
    ],
)

#cc_test(
#    name = "v2x_proxy_test",
#    srcs = [
#        "v2x_proxy_test.cc",
#    ],
#    deps = [
#        ":v2x_proxy",
#        "@com_google_googletest//:gtest_main",
#    ],
#)

cc_binary(
    name = "v2x",
    srcs = ["main.cc"],
    copts = V2X_COPTS,
    deps = [
        ":hik_proxy",
        ":v2x_proxy",
        "//cyber",
        "//modules/v2x/common:v2x_proxy_gflags",
        "//modules/v2x/proto:v2x_config_cc_proto",
    ],
)

install(
    name = "install",
    targets = [
        ":v2x",
    ],
    runtime_dest = "modules/v2x/v2x_proxy/app",
)

cc_library(
    name = "proxy",
    hdrs = ["proxy.h"],
    copts = V2X_COPTS,
)

cc_test(
    name = "utils_test",
    size = "small",
    timeout = "short",
    srcs = ["utils_test.cc"],
    deps = [
        ":utils",
        "//cyber",
        "//modules/map/hdmap:hdmap_util",
        "//modules/v2x/v2x_proxy/obu_interface:obu_interface_grpc_impl",
        "//modules/v2x/v2x_proxy/os_interface",
        "//modules/v2x/v2x_proxy/proto_adapter",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
