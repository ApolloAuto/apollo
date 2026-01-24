load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library", "apollo_plugin", "apollo_cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_profiler")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

apollo_plugin(
    name = "libonnx_multi_batch.so",
    description = ":plugins.xml",
    srcs = ["onnx_multi_batch_infer.cc"],
    hdrs = ["onnx_multi_batch_infer.h"],
    copts = PERCEPTION_COPTS + if_profiler() + ["-DENABLE_PROFILER=1"],
    deps = [
        "@local_config_cuda//cuda:cublas",
        "@local_config_cuda//cuda:cuda_headers",
        "@local_config_cuda//cuda:cudart",
        "@local_config_cuda//cuda:cudnn",
        "@local_config_cuda//cuda:cudnn_header",
        "@local_config_tensorrt//:tensorrt",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common/inference:apollo_perception_common_inference",
    ],
)

apollo_package(enable_source=False)
cpplint()
