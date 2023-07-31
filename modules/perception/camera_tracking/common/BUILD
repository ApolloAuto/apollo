load("@rules_cc//cc:defs.bzl", "cc_library")
load("@local_config_cuda//cuda:build_defs.bzl", "cuda_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cuda_library(
    name = "similar_cuda",
    srcs = ["similar.cu"],
    hdrs = ["similar.h"],
    deps = [
        ":camera_tracking_frame",
        "//cyber",
        "//modules/perception/common/camera/common:util",
        "//modules/perception/common/inference/utils:inference_gemm_lib",
        "@local_config_cuda//cuda:cudart",
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
        ":camera_tracking_frame",
        "//modules/perception/common/base",
        "//modules/perception/common/camera/common",
        "@eigen",
    ],
)

cc_library(
    name = "kalman_filter",
    srcs = ["kalman_filter.cc"],
    hdrs = ["kalman_filter.h"],
    deps = [
        "//cyber",
        "@eigen",
        "//modules/common/util:util_tool"
    ],
)

cc_library(
    name = "common",
    deps = [
        ":similar",
        ":kalman_filter",
    ],
)

cc_library(
    name = "camera_tracking_frame",
    hdrs = ["camera_tracking_frame.h"],
    deps = [
        "@eigen",
        "//modules/perception/common/base:blob",
        "//modules/perception/common/base",
        "//modules/perception/common/camera/common:data_provider",
        "//modules/perception/common/lib/interface:base_calibration_service",
    ],
)

apollo_package()
cpplint()
