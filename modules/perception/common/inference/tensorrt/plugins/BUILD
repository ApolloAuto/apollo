load("@local_config_cuda//cuda:build_defs.bzl", "cuda_library")
load("@local_config_tensorrt//:build_defs.bzl", "if_tensorrt_version_8")
load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "perception_inference_tensorrt_plugins",
    alwayslink = True,
    srcs = [
        ":dfmb_psroi_align_plugin",
        ":kernels_cuda",
        ":rcnn_proposal_plugin",
        ":rpn_proposal_ssd_plugin",
        ":perception_inference_tensorrt_plugins_argmax_cuda",
        ":perception_inference_tensorrt_plugins_leakyrelu_cuda",
        ":perception_inference_tensorrt_plugins_slice_cuda",
        ":perception_inference_tensorrt_plugins_softmax_cuda",
    ],
    hdrs = glob([
        "*.h",
    ]),
    linkstatic = True,
    deps = [
        "@local_config_cuda//cuda:cublas",
        "@local_config_cuda//cuda:cudart",
        "@local_config_cuda//cuda:cudnn",
    ],
)


cuda_library(
    name = "kernels_cuda",
    srcs = [
        "kernels.cu",
        "nms_cuda.cu",
    ],
    hdrs = ["kernels.h"],
    alwayslink = True,
    deps = [
        "//modules/perception/common/base:common",
        "@local_config_cuda//cuda:cudart",
    ],
)

cuda_library(
    name = "dfmb_psroi_align_plugin",
    srcs = ["dfmb_psroi_align_plugin.cu"],
    hdrs = ["dfmb_psroi_align_plugin.h"],
    alwayslink = True,
    deps = [
        ":kernels_cuda",
        "//cyber",
        "//modules/perception/common/inference/tensorrt:rt_common",
        "@local_config_cuda//cuda:cudart",
        "@local_config_tensorrt//:tensorrt",
        "//modules/perception/common/inference/tensorrt:rt_legacy",
    ],
)

cuda_library(
    name = "rcnn_proposal_plugin",
    srcs = ["rcnn_proposal_plugin.cu"],
    hdrs = ["rcnn_proposal_plugin.h"],
    alwayslink = True,
    deps = [
        ":kernels_cuda",
        "//modules/perception/common/inference/tensorrt:rt_common",
        "@local_config_cuda//cuda:cudart",
        "@local_config_tensorrt//:tensorrt",
        "//modules/perception/common/inference/tensorrt:rt_legacy",
    ],
)

cuda_library(
    name = "rpn_proposal_ssd_plugin",
    srcs = ["rpn_proposal_ssd_plugin.cu"],
    hdrs = ["rpn_proposal_ssd_plugin.h"],
    alwayslink = True,
    deps = [
        ":kernels_cuda",
        "//modules/perception/common/inference/tensorrt:rt_common",
        "@local_config_cuda//cuda:cudart",
        "@local_config_tensorrt//:tensorrt",
        "//modules/perception/common/inference/tensorrt:rt_legacy",
    ],
)

cuda_library(
    name = "perception_inference_tensorrt_plugins_slice_cuda",
    srcs = ["slice_plugin.cu"],
    hdrs = ["slice_plugin.h"],
    alwayslink = True,
    deps = [
        "//modules/perception/common/inference/tensorrt:rt_common",
        "@eigen",
        "@local_config_cuda//cuda:cudart",
        "@local_config_tensorrt//:tensorrt",
        "//modules/perception/common/inference/tensorrt:rt_legacy",
    ],
)

cuda_library(
    name = "perception_inference_tensorrt_plugins_argmax_cuda",
    srcs = ["argmax_plugin.cu"],
    hdrs = ["argmax_plugin.h"],
    alwayslink = True,
    deps = [
        "//modules/perception/common/inference/tensorrt:rt_common",
        "@eigen",
        "@local_config_cuda//cuda:cudart",
        "@local_config_tensorrt//:tensorrt",
        "//modules/perception/common/inference/tensorrt:rt_legacy",
    ],
)

cuda_library(
    name = "perception_inference_tensorrt_plugins_softmax_cuda",
    srcs = ["softmax_plugin.cu"],
    hdrs = ["softmax_plugin.h"],
    alwayslink = True,
    deps = [
        "//modules/perception/common/inference/tensorrt:rt_common",
        "@eigen",
        "@local_config_cuda//cuda:cudart",
        "@local_config_tensorrt//:tensorrt",
        "//modules/perception/common/inference/tensorrt:rt_legacy",
    ],
)

cuda_library(
    name = "perception_inference_tensorrt_plugins_leakyrelu_cuda",
    srcs = ["leakyReLU_plugin.cu"],
    hdrs = ["leakyReLU_plugin.h"],
    alwayslink = True,
    deps = [
        "//modules/perception/common/inference/tensorrt:rt_common",
        "@eigen",
        "@local_config_cuda//cuda:cudart",
        "@local_config_cuda//cuda:cudnn",
        "@local_config_tensorrt//:tensorrt",
        "//modules/perception/common/inference/tensorrt:rt_legacy",
    ],
)

cc_test(
    name = "slice_plugin_test",
    size = "small",
    srcs = ["slice_plugin_test.cc"],
    deps = [
        ":perception_inference_tensorrt_plugins",
        "//modules/perception/common/inference/tensorrt:rt_common",
        "@com_google_googletest//:gtest_main",
        "//modules/perception/common/inference/tensorrt:rt_legacy",
    ],
    linkstatic = True,
)

cc_test(
    name = "argmax_plugin_test",
    size = "small",
    srcs = ["argmax_plugin_test.cc"],
    deps = [
        ":perception_inference_tensorrt_plugins",
        "//modules/perception/common/inference/tensorrt:rt_common",
        "@com_google_googletest//:gtest_main",
        "@local_config_cuda//cuda:cublas",
        "//modules/perception/common/inference/tensorrt:rt_legacy",
    ],
    linkstatic = True,
)

apollo_package()
cpplint()
