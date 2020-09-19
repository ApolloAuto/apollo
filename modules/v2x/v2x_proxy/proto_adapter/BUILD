load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "proto_adapter",
    srcs = ["proto_adapter.cc"],
    hdrs = ["proto_adapter.h"],
    deps = [
        "//cyber",
        "//modules/map/hdmap",
        "//modules/v2x/common:v2x_proxy_gflags",
        "//modules/v2x/proto:v2x_car_status_cc_proto",
        "//modules/v2x/proto:v2x_junction_cc_proto",
        "//modules/v2x/proto:v2x_obu_rsi_cc_proto",
        "//modules/v2x/proto:v2x_obu_traffic_light_cc_proto",
        "//modules/v2x/proto:v2x_rsi_cc_proto",
        "//modules/v2x/proto:v2x_traffic_light_cc_proto",
        "@eigen",
    ],
)

cc_test(
    name = "proto_adapter_test",
    size = "small",
    timeout = "short",
    srcs = ["proto_adapter_test.cc"],
    deps = [
        ":proto_adapter",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
