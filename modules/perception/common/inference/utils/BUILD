load("//tools:apollo_package.bzl", "apollo_cc_test", "apollo_package")
load("//third_party/gpus:common.bzl", "gpu_library", "if_cuda", "if_rocm")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

gpu_library(
    name = "apollo_perception_common_utils",
    srcs = [
        "binary_data.cc",
        "cuda_util.cu",
        "gemm.cu",
        "resize.cu",
        "util.cc",
        "util.cu",
    ],
    hdrs = [
        "binary_data.h",
        "cuda_util.h",
        "gemm.h",
        "resize.h",
        "util.h",
    ],
    deps = [
        "//cyber",
        "//modules/perception/common/base:apollo_perception_common_base",
        "@boost",
        "@eigen",
        "@opencv//:imgcodecs",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
        "@local_config_cuda//cuda:cublas",
        "@local_config_tensorrt//:tensorrt",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
        "@local_config_rocm//rocm:migraphx",
        "@local_config_rocm//rocm:hipblas",
    ]),
    alwayslink = True,
)

apollo_cc_test(
    name = "inference_util_test",
    size = "small",
    srcs = ["util_test.cc"],
    data = [
        "//modules/perception/common/inference:inference_test_data",
    ],
    linkstatic = True,
    deps = [
        ":apollo_perception_common_utils",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "binary_data_test",
    size = "small",
    srcs = ["binary_data_test.cc"],
    data = [
        "//modules/perception/common/inference:inference_test_data",
    ],
    linkstatic = True,
    deps = [
        ":apollo_perception_common_utils",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()

cpplint()
