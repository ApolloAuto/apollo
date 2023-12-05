# AMD only, excluded for NVIDIA
load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools:cpplint.bzl", "cpplint")
load("//third_party/gpus:common.bzl", "gpu_library", "if_cuda", "if_rocm")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "mi_common",
    srcs = if_rocm(["mi_common.cc"]),
    hdrs = if_rocm(["mi_common.h"]),
    deps = [
        "//cyber",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/proto:rt_cc_proto",
        "@com_google_absl//:absl",
        "@local_config_rocm//rocm:migraphx",
        "@local_config_rocm//rocm:rocm_headers",
    ],
)

apollo_cc_test(
    name = "mi_common_test",
    size = "small",
    srcs = if_rocm(["mi_common_test.cc"]),
    deps = [
        ":mi_common",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_library(
    name = "mi_utils",
    srcs = if_rocm(["mi_utils.cc"]),
    hdrs = if_rocm(["mi_utils.h"]),
    data = [
        "//modules/perception/common/inference:inference_test_data",
    ],
    deps = [
        "//cyber",
        "//modules/perception/common/proto:rt_cc_proto",
    ],
)

apollo_cc_test(
    name = "mi_utils_test",
    size = "small",
    srcs = if_rocm(["mi_utils_test.cc"]),
    deps = [
        ":mi_utils",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_library(
    name = "mi_net",
    srcs = if_rocm(["mi_net.cc"]),
    hdrs = if_rocm([
        "mi_common.h",
        "mi_net.h",
        "mi_utils.h",
    ]),
    deps = [
        ":mi_common",
        ":mi_utils",
        "//cyber",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/inference:inference_lib",
        "//modules/perception/common/inference/migraphx/plugins:perception_inference_migraphx_plugins",
        "@com_google_protobuf//:protobuf",
    ],
)

apollo_cc_test(
    name = "mi_net_test",
    size = "medium",
    srcs = if_rocm(["mi_net_test.cc"]),
    data = [
        "//modules/perception/common/inference:inference_test_data",
    ],
    deps = [
        ":mi_net",
        "@com_google_googletest//:gtest_main",
    ],
)


apollo_package()

cpplint()
