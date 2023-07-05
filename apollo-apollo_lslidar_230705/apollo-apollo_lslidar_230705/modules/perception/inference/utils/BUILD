load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//third_party/gpus:common.bzl", "gpu_library", "if_cuda", "if_rocm")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

gpu_library(
    name = "inference_cuda_util_cuda_lib",
    srcs = ["cuda_util.cu"],
    hdrs = ["cuda_util.h"],
    deps = [
        "//cyber",
        "@boost",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
        "@local_config_rocm//rocm:rocm_headers",
        "@local_config_rocm//rocm:hipblas",
    ]),
)

cc_library(
    name = "inference_cuda_util_lib",
    srcs = [":inference_cuda_util_cuda_lib"],
    hdrs = ["cuda_util.h"],
    linkstatic = 1,
    deps = if_cuda([
        "@local_config_tensorrt//:tensorrt",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
        "@local_config_rocm//rocm:migraphx",
    ]) + [
        "@opencv//:imgcodecs"
    ],
)

gpu_library(
    name = "inference_util_cuda_lib",
    srcs = ["util.cu"],
    hdrs = ["util.h"],
    deps = [
        "//cyber",
        "//modules/perception/base",
        "@eigen",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
    ]),
)

cc_library(
    name = "inference_util_lib",
    srcs = [
        "util.cc",
        ":inference_util_cuda_lib",
    ],
    hdrs = ["util.h"],
    linkstatic = 1,
    deps = [
        "//modules/perception/base:blob",
        "//modules/perception/base:box",
        "//modules/perception/base:image",
        "//modules/perception/base:point_cloud",
    ] + if_cuda([
        "@local_config_tensorrt//:tensorrt",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
        "@local_config_rocm//rocm:migraphx",
    ]) + [
        "@opencv//:imgcodecs",
    ],
)

gpu_library(
    name = "inference_resize_cuda_lib",
    srcs = ["resize.cu"],
    hdrs = ["resize.h"],
    deps = [
        ":inference_cuda_util_lib",
        ":inference_util_lib",
        "//cyber",
        "//modules/perception/base:blob",
        "//modules/perception/base:image",
    ] + if_cuda([
        "@local_config_cuda//cuda:cublas",
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hipblas",
        "@local_config_rocm//rocm:hip",
    ]),
)

cc_library(
    name = "inference_resize_lib",
    srcs = [":inference_resize_cuda_lib"],
    hdrs = ["resize.h"],
    linkstatic = 1,
    deps = if_cuda([
        "@local_config_tensorrt//:tensorrt",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
        "@local_config_rocm//rocm:migraphx",
    ]) + [
        "@opencv//:imgcodecs",
    ],
)

gpu_library(
    name = "inference_gemm_cuda_lib",
    srcs = ["gemm.cu"],
    hdrs = ["gemm.h"],
    deps = [
        ":inference_cuda_util_lib",
        ":inference_util_lib",
        "//cyber",
        "//modules/perception/base:blob",
        "//modules/perception/base:image",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
    ]),
)

cc_library(
    name = "inference_gemm_lib",
    srcs = [":inference_gemm_cuda_lib"],
    hdrs = ["gemm.h"],
    linkstatic = 1,
    deps = if_cuda([
        "@local_config_tensorrt//:tensorrt",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
        "@local_config_rocm//rocm:migraphx",
    ]) + [
        "@opencv//:imgcodecs",
    ],
)

cc_test(
    name = "inference_util_test",
    size = "small",
    srcs = ["util_test.cc"],
    data = [
        "//modules/perception/inference:inference_test_data",
    ],
    deps = [
        ":inference_util_lib",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

cc_library(
    name = "inference_binary_data_lib",
    srcs = ["binary_data.cc"],
    hdrs = ["binary_data.h"],
    deps = [
        "//modules/perception/base:blob",
    ] + if_cuda([
        "@local_config_tensorrt//:tensorrt",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
        "@local_config_rocm//rocm:migraphx",
    ]) + [
        "@opencv//:imgcodecs",
    ],
)

cc_test(
    name = "binary_data_test",
    size = "small",
    srcs = ["binary_data_test.cc"],
    data = [
        "//modules/perception/inference:inference_test_data",
    ],
    deps = [
        ":inference_binary_data_lib",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

cpplint()
