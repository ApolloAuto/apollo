load("//tools:cpplint.bzl", "cpplint")
load("//tools:cuda_library.bzl", "cuda_library")

package(default_visibility = ["//visibility:public"])

cuda_library(
    name = "similar_cuda",
    srcs = ["similar.cu"],
    hdrs = ["similar.h"],
    deps = [
        "//cyber",
        "//modules/perception/camera/common:camera_frame",
        "//modules/perception/inference/utils:inference_gemm_lib",
	"//modules/prediction/proto:feature_proto",
	"@cuda",
    ],
)

cc_library(
    name = "similar",
    srcs = [
        "similar.cc",
        ":similar_cuda",
    ],
    hdrs = ["similar.h"],
    deps = [
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "@eigen",
    ],
)

cc_library(
    name = "half_circle_angle",
    srcs = ["half_circle_angle.cc"],
    hdrs = ["half_circle_angle.h"],
    deps = [
        "//modules/perception/camera/common",
    ],
)

cc_library(
    name = "kalman_filter",
    srcs = ["kalman_filter.cc"],
    hdrs = ["kalman_filter.h"],
    deps = [
        "//cyber",
        "@eigen",
    ],
)

cc_library(
    name = "common",
    deps = [
        ":half_circle_angle",
        ":kalman_filter",
        ":similar",
    ],
)

cpplint()
