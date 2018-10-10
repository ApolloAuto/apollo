load("//tools:cpplint.bzl", "cpplint")
load("//tools:cuda_library.bzl", "cuda_library")

package(default_visibility = ["//visibility:public"])

cuda_library(
    name = "inference_cuda_util_cuda_lib",
    srcs = [
        "cuda_util.cu",
    ],
    hdrs = [
        "cuda_util.h",
    ],
    deps = [
        "//cybertron",
        "@cuda",
    ],
)

cc_library(
    name = "inference_cuda_util_lib",
    srcs = [
        ":inference_cuda_util_cuda_lib",
    ],
    hdrs = ["cuda_util.h"],
    linkstatic = 1,
    linkopts = ["-lopencv_core -lopencv_imgproc -lopencv_highgui -lnvinfer_plugin -lcaffe"],
    deps = [],
)

cuda_library(
    name = "inference_util_cuda_lib",
    srcs = [
        "util.cu",
    ],
    hdrs = [
        "util.h",
    ],
    deps = [
        "//cybertron",
        "@cuda",
        "@eigen",
    ],
)

cc_library(
    name = "inference_util_lib",
    srcs = [
        "util.cc",
        ":inference_util_cuda_lib",
    ],
    hdrs = ["util.h"],
    linkstatic = 1,
    linkopts = ["-lopencv_core -lopencv_imgproc -lopencv_highgui -lnvinfer_plugin -lcaffe"],
    deps = [
        "//modules/perception/base:blob",
        "//modules/perception/base:image",
        "//modules/perception/base:box",
        "//modules/perception/base:point_cloud",
    ],
)

cuda_library(
    name = "inference_resize_cuda_lib",
    srcs = [
        "resize.cu",
    ],
    hdrs = [
        "resize.h",
    ],
    deps = [
        "//cybertron",
        ":inference_util_lib",
        ":inference_cuda_util_lib",
        "//modules/perception/base:blob",
        "//modules/perception/base:image",
        "@cuda",
    ],
)

cuda_library(
    name = "inference_gemm_cuda_lib",
    srcs = [
        "gemm.cu",
    ],
    hdrs = [
        "gemm.h",
    ],
    deps = [
        "//cybertron",
        ":inference_util_lib",
        ":inference_cuda_util_lib",
        "//modules/perception/base:blob",
        "//modules/perception/base:image",
        "@cuda",
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
        "@gtest//:main",
    ],
)

cc_library(
    name = "inference_binary_data_lib",
    srcs = [
        "binary_data.cc",
    ],
    hdrs = ["binary_data.h"],
    linkopts = ["-lopencv_core -lopencv_imgproc -lopencv_highgui -lnvinfer_plugin -lcaffe"],
    deps = [
        "//modules/perception/base:blob",
    ],
)

cc_test(
    name = "inference_binary_data_test",
    size = "small",
    srcs = ["binary_data_test.cc"],
    data = [
        "//modules/perception/inference:inference_test_data",
    ],
    deps = [
        ":inference_binary_data_lib",
        "@gtest//:main",
    ],
)

cpplint()