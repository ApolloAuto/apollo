load("@local_config_cuda//cuda:build_defs.bzl", "cuda_library")
load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cuda_library(
    name = "inference_cuda_util_cuda_lib",
    srcs = ["cuda_util.cu"],
    hdrs = ["cuda_util.h"],
    alwayslink = True,
    deps = [
        "//cyber",
        "@boost",
        "@local_config_cuda//cuda:cudart",
    ],
)

cc_library(
    name = "inference_cuda_util_lib",
    srcs = [":inference_cuda_util_cuda_lib"],
    hdrs = ["cuda_util.h"],
    alwayslink = True,
    linkstatic = 1,
    deps = [
        "@local_config_tensorrt//:tensorrt",
        "@opencv//:imgcodecs",
    ],
)

cuda_library(
    name = "inference_util_cuda_lib",
    srcs = ["util.cu"],
    hdrs = ["util.h"],
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/perception/common/base",
        "@eigen",
        "@local_config_cuda//cuda:cudart",
    ],
)

cc_library(
    name = "inference_util_lib",
    srcs = [
        "util.cc",
        ":inference_util_cuda_lib",
    ],
    alwayslink = True,
    hdrs = ["util.h"],
    linkstatic = 1,
    deps = [
        "//modules/perception/common/base:blob",
        "//modules/perception/common/base:box",
        "//modules/perception/common/base:image",
        "//modules/perception/common/base:point_cloud",
        "@local_config_tensorrt//:tensorrt",
        "@opencv//:imgcodecs",
    ],
)

cuda_library(
    name = "inference_resize_cuda_lib",
    srcs = ["resize.cu"],
    hdrs = ["resize.h"],
    alwayslink = True,
    deps = [
        ":inference_cuda_util_lib",
        ":inference_util_lib",
        "//cyber",
        "//modules/perception/common/base:blob",
        "//modules/perception/common/base:image",
        "@local_config_cuda//cuda:cublas",
        "@local_config_cuda//cuda:cudart",
    ],
)

cc_library(
    name = "inference_resize_lib",
    srcs = [":inference_resize_cuda_lib"],
    hdrs = ["resize.h"],
    alwayslink = True,
    linkstatic = 1,
    deps = [
        "@local_config_tensorrt//:tensorrt",
        "@opencv//:imgcodecs",
    ],
)

cuda_library(
    name = "inference_gemm_cuda_lib",
    srcs = ["gemm.cu"],
    hdrs = ["gemm.h"],
    alwayslink = True,
    deps = [
        ":inference_cuda_util_lib",
        ":inference_util_lib",
        "//cyber",
        "//modules/perception/common/base:blob",
        "//modules/perception/common/base:image",
        "@local_config_cuda//cuda:cudart",
    ],
)

cc_library(
    name = "inference_gemm_lib",
    srcs = [":inference_gemm_cuda_lib"],
    hdrs = ["gemm.h"],
    alwayslink = True,
    linkstatic = 1,
    deps = [
        "@local_config_tensorrt//:tensorrt",
        "@opencv//:imgcodecs",
    ],
)

cc_test(
    name = "inference_util_test",
    size = "small",
    srcs = ["util_test.cc"],
    data = [
        "//modules/perception/common/inference:inference_test_data",
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
    alwayslink = True,
    deps = [
        "//modules/perception/common/base:blob",
        "@local_config_tensorrt//:tensorrt",
        "@opencv//:imgcodecs",
    ],
)

cc_test(
    name = "binary_data_test",
    size = "small",
    srcs = ["binary_data_test.cc"],
    data = [
        "//modules/perception/common/inference:inference_test_data",
    ],
    deps = [
        ":inference_binary_data_lib",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_package()
cpplint()
