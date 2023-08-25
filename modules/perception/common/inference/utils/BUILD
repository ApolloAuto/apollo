load("@local_config_cuda//cuda:build_defs.bzl", "cuda_library")
load("//tools:apollo_package.bzl", "apollo_cc_test", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cuda_library(
    name = "apollo_perception_common_utils",
    srcs = [
        "cuda_util.cu",
        "util.cu",
        "util.cc",
        "resize.cu",
        "gemm.cu",
        "binary_data.cc",
    ],
    hdrs = [
        "cuda_util.h",
        "util.h",
        "resize.h",
        "gemm.h",
        "binary_data.h",
    ],
    deps = [
        "//cyber",
        "//modules/perception/common/base:apollo_perception_common_base",
        "@eigen",
        "@boost",
        "@local_config_cuda//cuda:cudart",
        "@local_config_cuda//cuda:cublas",
        "@local_config_tensorrt//:tensorrt",
        "@opencv//:imgcodecs",
    ],
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
