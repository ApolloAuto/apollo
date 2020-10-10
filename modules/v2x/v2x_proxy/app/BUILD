load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "utils",
    srcs = ["utils.cc"],
    hdrs = ["utils.h"],
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
    deps = [
        ":utils",
        "//cyber",
        "//modules/map/hdmap:hdmap_util",
        "//modules/v2x/v2x_proxy/obu_interface:obu_interface_grpc_impl",
        "//modules/v2x/v2x_proxy/os_interface",
        "//modules/v2x/v2x_proxy/proto_adapter",
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
    deps = [
        ":v2x_proxy",
    ],
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
