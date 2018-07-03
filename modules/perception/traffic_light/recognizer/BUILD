load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_traffic_light_recognizer",
    srcs = [
        "classify.cc",
        "unity_recognize.cc",
    ],
    hdrs = [
        "classify.h",
        "unity_recognize.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common/proto:error_code_proto",
        "//modules/common/proto:header_proto",
        "//modules/map/proto:map_proto",
        "//modules/perception/proto:perception_proto",
        "//modules/perception/proto/traffic_light:recognizer_config_lib_proto",
        "//modules/perception/traffic_light/base",
        "//modules/perception/traffic_light/interface",
    ],
)

cc_test(
    name = "unity_recognize_test",
    size = "small",
    srcs = [
        "unity_recognize_test.cc",
    ],
    data = [
        "//modules/perception:perception_data",
        "//modules/perception:perception_model",
        "//modules/perception/conf:perception_config",
    ],
    deps = [
        "perception_traffic_light_recognizer",
        "//modules/perception/lib/base",
        "@gtest//:main",
    ],
)

cpplint()
