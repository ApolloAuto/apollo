load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//third_party/gpus:common.bzl", "gpu_library", "if_cuda", "if_rocm")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

apollo_cc_library(
    name = "perception_inference_migraphx_plugins",
    hdrs = if_rocm(["perception_inference_migraphx_plugins.h"]),
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
    mandatory = False,
    srcs = if_rocm([
        "kernels.cu",
        "nms_cuda.cu",
    ]),
    hdrs = if_rocm(["kernels.h"]),
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

apollo_cc_library(
    name = "plugin",
    hdrs = ["plugin.h"],
)

gpu_library(
    name = "dfmb_psroi_align_plugin",
    srcs = if_rocm(["dfmb_psroi_align_plugin.cu"]),
    hdrs = if_rocm(["dfmb_psroi_align_plugin.h"]),
    deps = [
        ":kernels_cuda",
        ":plugin",
        "//cyber",
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
    mandatory = False,
    srcs = if_rocm(["rcnn_proposal_plugin.cu"]),
    hdrs = if_rocm(["rcnn_proposal_plugin.h"]),
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
    mandatory = False,
    srcs = if_rocm(["rpn_proposal_ssd_plugin.cu"]),
    hdrs = if_rocm(["rpn_proposal_ssd_plugin.h"]),
    deps = if_rocm([
        ":kernels_cuda",
        ":plugin",
        "//modules/perception/common/inference/migraphx:mi_common",
    ]) + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
        "@local_config_rocm//rocm:rocm_headers",
    ]),
)

apollo_package()
cpplint()
