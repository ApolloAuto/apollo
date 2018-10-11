load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "conti_ars_detector_test",
    size = "small",
    srcs = [
        "conti_ars_detector_test.cc",
    ],
    deps = [
        ":conti_ars_detector",
        "@gtest//:main",
    ],
)

cc_library(
    name = "conti_ars_detector",
    srcs = [
        "conti_ars_detector.cc",
    ],
    hdrs = [
        "conti_ars_detector.h",
    ],
    deps = [
        "//modules/perception/base:base",
        "//modules/perception/lib/singleton:singleton",
        "//modules/perception/lib/config_manager:config_manager",
        "//modules/perception/lib/registerer:registerer",
        "//modules/perception/common/geometry:roi_filter",
        "//modules/perception/radar/common:types",
        "//modules/perception/radar/common:radar_util",
        "//modules/perception/radar/lib/interface:base_detector",
        "//modules/perception/proto:perception_config_schema_proto",
        "//modules/drivers/proto:sensor_proto",
        "@eigen",
    ],
)

cpplint()
