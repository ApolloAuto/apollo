load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//third_party/gpus:common.bzl", "gpu_library", "if_cuda", "if_rocm")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "perception_inference_migraphx_plugins",
    hdrs = ["perception_inference_migraphx_plugins.h"],
    linkstatic = True,
    deps = [
        ":dfmb_psroi_align_plugin",
        ":kernels_cuda",
        ":rcnn_proposal_plugin",
        ":rpn_proposal_ssd_plugin",
        "//modules/perception/common/base:apollo_perception_common_base",
    ] + if_cuda([
        "@local_config_cuda//cuda:cublas",
        "@local_config_cuda//cuda:cudart",
        "@local_config_cuda//cuda:cudnn",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hipblas",
        "@local_config_rocm//rocm:hip",
        "@local_config_rocm//rocm:rocm_headers",
    ]),
)

gpu_library(
    name = "kernels_cuda",
    srcs = [
        "kernels.cu",
        "nms_cuda.cu",
    ],
    hdrs = ["kernels.h"],
    deps = [
        "//modules/perception/common/base:apollo_perception_common_base",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
        "@local_config_rocm//rocm:rocm_headers",
    ]),
    alwayslink = True,
)

cc_library(
    name = "plugin",
    hdrs = ["plugin.h"],
)

gpu_library(
    name = "dfmb_psroi_align_plugin",
    srcs = ["dfmb_psroi_align_plugin.cu"],
    hdrs = ["dfmb_psroi_align_plugin.h"],
    deps = [
        ":kernels_cuda",
        ":plugin",
        "//cyber/common:log",
        "//modules/perception/common/inference/migraphx:mi_common",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
        "@local_config_rocm//rocm:rocm_headers",
    ]),
)

gpu_library(
    name = "rcnn_proposal_plugin",
    srcs = ["rcnn_proposal_plugin.cu"],
    hdrs = ["rcnn_proposal_plugin.h"],
    deps = [
        ":kernels_cuda",
        ":plugin",
        "//modules/perception/common/inference/migraphx:mi_common",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
        "@local_config_rocm//rocm:rocm_headers",
    ]),
)

gpu_library(
    name = "rpn_proposal_ssd_plugin",
    srcs = ["rpn_proposal_ssd_plugin.cu"],
    hdrs = ["rpn_proposal_ssd_plugin.h"],
    deps = [
        ":kernels_cuda",
        ":plugin",
        "//modules/perception/common/inference/migraphx:mi_common",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
        "@local_config_rocm//rocm:rocm_headers",
    ]),
)

cpplint()
