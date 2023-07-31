load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "inference_files",
    srcs = glob([
        "inference_test_data/**",
    ])
)

cc_library(
    name = "layer_lib",
    srcs = ["layer.cc"],
    hdrs = ["layer.h"],
    alwayslink = True,
    deps = [
        "//modules/perception/common/base:blob",
    ],
)

cc_test(
    name = "layer_test",
    size = "small",
    srcs = ["layer_test.cc"],
    deps = [
        ":layer_lib",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

cc_library(
    name = "inference_lib",
    srcs = ["inference.cc"],
    hdrs = ["inference.h"],
    alwayslink = True,
    deps = [
        "//modules/perception/common/base:blob",
        "@boost",
    ],
)

cc_test(
    name = "inference_test",
    size = "small",
    srcs = ["inference_test.cc"],
    deps = [
        ":inference_lib",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

cc_library(
    name = "inference_factory",
    srcs = ["inference_factory.cc"],
    hdrs = ["inference_factory.h"],
    alwayslink = True,
    deps = [
        ":inference_lib",
        "//modules/perception/common/inference/libtorch:torch_net",
        "//modules/perception/common/inference/onnx:libtorch_obstacle_detector",
        "//modules/perception/common/inference/paddlepaddle:paddle_net",
        "//modules/perception/common/inference/tensorrt:rt_net",
        "//modules/perception/common/inference/utils:inference_util_lib",
        "//modules/perception/common/proto:model_info_cc_proto",
        "//modules/perception/common/inference/tensorrt:rt_legacy",
        "@local_config_tensorrt//:tensorrt",
    ],
)

cc_test(
    name = "inference_factory_test",
    size = "small",
    srcs = ["inference_factory_test.cc"],
    deps = [
        ":inference_factory",
        "@com_google_googletest//:gtest_main",
        "//modules/perception/common/inference/tensorrt:rt_legacy",
        "@local_config_tensorrt//:tensorrt",
    ],
    linkstatic = True,
)

cc_library(
    name = "model_util",
    srcs = ["model_util.cc"],
    hdrs = ["model_util.h"],
    deps = [
        "//modules/perception/common/proto:model_info_cc_proto",
        "@com_google_protobuf//:protobuf",
    ],
)

filegroup(
    name = "inference_test_data",
    srcs = glob(["inference_test_data/**"]),
)

apollo_package()
cpplint()
