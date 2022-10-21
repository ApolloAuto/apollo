# AMD only, excluded for NVIDIA
load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "mi_common",
    srcs = ["mi_common.cc"],
    hdrs = ["mi_common.h"],
    deps = [
        "//cyber",
        "//modules/perception/base:common",
        "//modules/perception/proto:rt_cc_proto",
        "@com_google_absl//:absl",
        "@local_config_rocm//rocm:migraphx",
        "@local_config_rocm//rocm:rocm_headers",
    ],
)

cc_test(
    name = "mi_common_test",
    size = "small",
    srcs = ["mi_common_test.cc"],
    deps = [
        ":mi_common",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "mi_utils",
    srcs = ["mi_utils.cc"],
    hdrs = ["mi_utils.h"],
    data = [
        "//modules/perception/inference:inference_test_data",
    ],
    deps = [
        "//cyber",
        "//modules/perception/proto:rt_cc_proto",
    ],
)

cc_test(
    name = "mi_utils_test",
    size = "small",
    srcs = ["mi_utils_test.cc"],
    deps = [
        ":mi_utils",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "mi_net",
    srcs = ["mi_net.cc"],
    hdrs = [
        "mi_common.h",
        "mi_utils.h",
        "mi_net.h",
    ],
    deps = [
        ":mi_common",
        ":mi_utils",
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/inference:inference_lib",
        "//modules/perception/inference/migraphx/plugins:perception_inference_migraphx_plugins",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_test(
    name = "mi_net_test",
    size = "medium",
    srcs = ["mi_net_test.cc"],
    data = [
        "//modules/perception/inference:inference_test_data",
    ],
    deps = [
        ":mi_net",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
