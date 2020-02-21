load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pbf_gatekeeper",
    srcs = ["pbf_gatekeeper.cc"],
    hdrs = ["pbf_gatekeeper.h"],
    deps = [
        "//modules/common/util",
        "//modules/perception/fusion/base:base_init_options",
        "//modules/perception/fusion/base:scene",
        "//modules/perception/fusion/base:track",
        "//modules/perception/fusion/lib/gatekeeper/pbf_gatekeeper/proto:pbf_gatekeeper_config_proto",
        "//modules/perception/fusion/lib/interface",
        "//modules/perception/lib/config_manager",
    ],
)

cc_test(
    name = "pbf_gatekeeper_test",
    size = "small",
    srcs = ["pbf_gatekeeper_test.cc"],
    deps = [
        ":pbf_gatekeeper",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/fusion/base:sensor",
        "//modules/perception/fusion/base:track",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
