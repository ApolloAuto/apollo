load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_traffic_light_preprocessor",
    srcs = [
        "tl_preprocessor.cc",
    ],
    hdrs = [
        "tl_preprocessor.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common/proto:error_code_proto",
        "//modules/common/proto:header_proto",
        "//modules/perception/proto/traffic_light:preprocessor_config_lib_proto",
        "//modules/perception/traffic_light/base",
        "//modules/perception/traffic_light/interface",
        "//modules/perception/traffic_light/projection:perception_traffic_light_projection",
    ],
)

cc_test(
    name = "tl_preprocessor_test",
    size = "small",
    srcs = [
        "tl_preprocessor_test.cc",
    ],
    data = [
        "//modules/perception:perception_data",
        "//modules/perception:perception_model",
        "//modules/perception/conf:perception_config",
    ],
    deps = [
        ":perception_traffic_light_preprocessor",
        "//modules/perception/lib/base",
        "@gtest//:main",
    ],
)

cpplint()
