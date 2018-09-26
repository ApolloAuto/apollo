load("//tools:cpplint.bzl", "cpplint")
load("//tools:cuda_library.bzl", "cuda_library")

package(default_visibility = ["//visibility:public"])

cuda_library(
    name = "inference_util_cuda_lib",
    srcs = [
        "util.cu",
    ],
    hdrs = [
        "util.h",
    ],
    deps = [
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
    deps = [
        "//modules/perception/base:blob",
    ],
)

cc_test(
    name = "inference_util_test",
    size = "small",
    data = [
        "//modules/perception/inference:inference_test_data",
    ],
    srcs = ["util_test.cc"],
    linkopts = ["-lopencv_core -lopencv_imgproc -lopencv_highgui -lnvinfer_plugin -lcaffe"],
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
    deps = [
        "//modules/perception/base:blob",
    ],
)

cc_test(
    name = "inference_binary_data_test",
    size = "small",
    data = [
        "//modules/perception/inference:inference_test_data",
    ],
    srcs = ["binary_data_test.cc"],
    linkopts = ["-lopencv_core -lopencv_imgproc -lopencv_highgui -lnvinfer_plugin -lcaffe"],
    deps = [
        ":inference_binary_data_lib",
        "@gtest//:main",
    ],
)

cpplint()
