load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
load("//third_party/gpus:common.bzl", "gpu_library", "if_cuda", "if_rocm")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "inference_files",
    srcs = glob([
        "inference_test_data/**",
    ]),
)

apollo_cc_library(
    name = "layer_lib",
    srcs = ["layer.cc"],
    hdrs = ["layer.h"],
    deps = [
        "//modules/perception/common/base:apollo_perception_common_base",
    ],
)

apollo_cc_library(
    name = "inference_lib",
    srcs = ["inference.cc"],
    hdrs = ["inference.h"],
    deps = [
        "//modules/perception/common/base:apollo_perception_common_base",
        "@libtorch_gpu",
        "@boost",
    ],
)

apollo_cc_test(
    name = "layer_test",
    size = "small",
    srcs = ["layer_test.cc"],
    linkstatic = True,
    deps = [
        ":layer_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "inference_test",
    size = "small",
    srcs = ["inference_test.cc"],
    linkstatic = True,
    deps = [
        ":inference_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

gpu_library(
    name = "perception_inference_operators_cuda",
    srcs = [
        "operators/roipooling_layer.cc",
        "operators/roipooling_layer.cu",
    ],
    hdrs = ["operators/roipooling_layer.h"],
    deps = [
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/inference:layer_lib",
        "@eigen",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
    ]),
    alwayslink = True,
)

apollo_cc_library(
    name = "apollo_perception_common_inference",
    srcs = [
        "inference_factory.cc",
        "libtorch/torch_net.cc",
        "model_util.cc",
        "onnx/libtorch_obstacle_detector.cc",
        "onnx/onnx_single_batch_infer.cc",
        "paddlepaddle/paddle_net.cc",
    ],
    hdrs = [
        "inference_factory.h",
        "libtorch/torch_net.h",
        "model_util.h",
        "onnx/libtorch_obstacle_detector.h",
        "onnx/onnx_single_batch_infer.h",
        "paddlepaddle/paddle_net.h",
    ],
    deps = [
        ":inference_lib",
        ":layer_lib",
        ":perception_inference_operators_cuda",
        "//cyber",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/inference/tensorrt:apollo_perception_inference_rt_net",
        "//modules/perception/common/inference/utils:apollo_perception_common_utils",
        "//modules/perception/common/proto:model_info_cc_proto",
        "//modules/perception/common/proto:rt_cc_proto",
        "@boost",
        "@com_google_absl//:absl",
        "@com_google_protobuf//:protobuf",
        "@opencv//:imgcodecs",
        "@libtorch_gpu",
    ] + select({
        "@platforms//cpu:x86_64": ["@paddleinference-x86_64//:paddleinference_lib"],
        "@platforms//cpu:aarch64": ["@paddleinference-aarch64//:paddleinference_lib"],
    }) + if_cuda([
        "@local_config_cuda//cuda:cudart",
        "@local_config_cuda//cuda:cublas",
        "@local_config_cuda//cuda:cuda_headers",
        "@local_config_cuda//cuda:cudnn",
        "@local_config_cuda//cuda:cudnn_header",
        "@local_config_tensorrt//:tensorrt",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
        "@local_config_rocm//rocm:hipblas",
        "@local_config_rocm//rocm:rocm_headers",
        "@local_config_rocm//rocm:migraphx",
    ]),
)

filegroup(
    name = "inference_test_data",
    srcs = glob(["inference_test_data/**"]),
)

filegroup(
    name = "libtorch_test_files",
    srcs = glob([
        "libtorch/test_data/**",
    ]),
)

filegroup(
    name = "onnx_test_files",
    srcs = glob([
        "onnx/testdata/**",
    ]),
)

apollo_package()

cpplint()
