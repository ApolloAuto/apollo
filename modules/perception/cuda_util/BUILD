load("//tools:cpplint.bzl", "cpplint")
load("//tools:cuda_library.bzl", "cuda_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "cuda_util",
    srcs = [
        ":integrated_cuda_util",
    ],
    hdrs = glob([
        "*.h",
    ]),
    linkstatic = True,
    deps = [
        "//modules/perception/obstacle/camera/lane_post_process/common:connected_component",
        "@caffe//:lib",
        "@cuda",
    ],
)

cuda_library(
    name = "integrated_cuda_util",
    srcs = [
        "block_uf.cu",
        "connected_component_gpu.cu",
        "network.cu",
        "region_output.cc",
        "region_output.cu",
        "undistortion.cu",
        "util.cu",
    ],
    hdrs = [
        "block_uf.h",
        "connected_component_gpu.h",
        "network.h",
        "region_output.h",
        "texture.h",
        "undistortion.h",
        "util.h",
    ],
    deps = [
        "@cuda",
        "@eigen",
    ],
)

cpplint()
