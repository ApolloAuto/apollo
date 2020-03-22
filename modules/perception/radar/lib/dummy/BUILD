load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "dummy_algorithms_test",
    size = "small",
    srcs = ["dummy_algorithms_test.cc"],
    deps = [
        ":dummy_algorithms",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "dummy_algorithms",
    srcs = ["dummy_algorithms.cc"],
    hdrs = ["dummy_algorithms.h"],
    deps = [
        "//modules/drivers/proto:sensor_proto",
        "//modules/perception/base",
        "//modules/perception/common/geometry:roi_filter",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lib/registerer",
        "//modules/perception/proto:perception_config_schema_proto",
        "//modules/perception/radar/common:types",
        "//modules/perception/radar/lib/interface:base_detector",
        "//modules/perception/radar/lib/interface:base_preprocessor",
        "//modules/perception/radar/lib/interface:base_roi_filter",
    ],
)

cpplint()
