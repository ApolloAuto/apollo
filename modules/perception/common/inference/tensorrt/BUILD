load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "rt_legacy",
    hdrs = ["rt_legacy.h"],
    alwayslink = True,
)

cc_library(
    name = "rt_common",
    srcs = ["rt_common.cc"],
    hdrs = ["rt_common.h"],
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/perception/common/base:common",
        "//modules/perception/common/proto:rt_cc_proto",
        "@com_google_absl//:absl",
        "@local_config_cuda//cuda:cudnn_header",
        "//modules/perception/common/inference/tensorrt:rt_legacy",
        "@local_config_tensorrt//:tensorrt",
    ],
)

cc_test(
    name = "rt_common_test",
    size = "small",
    srcs = ["rt_common_test.cc"],
    deps = [
        ":rt_common",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "rt_utils",
    srcs = ["rt_utils.cc"],
    hdrs = ["rt_utils.h"],
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/perception/common/proto:rt_cc_proto",
        "@local_config_tensorrt//:tensorrt",
        "//modules/perception/common/inference/tensorrt:rt_legacy",
    ],
)

cc_test(
    name = "rt_utils_test",
    size = "small",
    srcs = ["rt_utils_test.cc"],
    deps = [
        ":rt_utils",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

cc_library(
    name = "rt_net",
    srcs = ["rt_net.cc"],
    alwayslink = True,
    hdrs = [
        "entropy_calibrator.h",
        "rt_net.h",
    ],
    deps = [
        ":batch_stream",
        ":entropy_calibrator",
        ":rt_common",
        ":rt_utils",
        "//cyber",
        "//modules/perception/common/base",
        "//modules/perception/common/inference:inference_lib",
        "//modules/perception/common/inference/tensorrt/plugins:perception_inference_tensorrt_plugins",
        "@com_google_protobuf//:protobuf",
        "@local_config_tensorrt//:tensorrt",
        "@opencv//:imgcodecs",
        "//modules/perception/common/inference/tensorrt:rt_legacy",
    ],
)

cc_test(
    name = "rt_net_test",
    size = "small",
    srcs = ["rt_net_test.cc"],
    data = [
        "//modules/perception/common/inference:inference_test_data",
    ],
    tags = ["exclude"],
    deps = [
        ":rt_net",
        "//modules/perception/common/inference/utils:inference_util_cuda_lib",
        "@atlas//:cblas",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

cc_library(
    name = "entropy_calibrator",
    srcs = ["entropy_calibrator.cc"],
    hdrs = ["entropy_calibrator.h"],
    alwayslink = True,
    deps = [
        ":batch_stream",
        ":rt_utils",
        "@com_google_protobuf//:protobuf",
        "@local_config_cuda//cuda:cudart",
        "@local_config_tensorrt//:tensorrt",
        "@opencv//:imgcodecs",
        "//modules/perception/common/inference/tensorrt:rt_legacy",
    ],
)

cc_test(
    name = "entropy_calibrator_test",
    size = "small",
    srcs = ["entropy_calibrator_test.cc"],
    copts = ["-fno-access-control"],
    data = [
        "//modules/perception/common/inference:inference_test_data",
    ],
    deps = [
        ":entropy_calibrator",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

cc_library(
    name = "batch_stream",
    srcs = ["batch_stream.cc"],
    hdrs = ["batch_stream.h"],
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/perception/common/proto:rt_cc_proto",
        "@com_google_absl//:absl",
        "@com_google_protobuf//:protobuf",
        "@opencv//:imgcodecs",
        "@local_config_cuda//cuda:cuda_headers",
        "//modules/perception/common/inference/tensorrt:rt_legacy",
        "@local_config_tensorrt//:tensorrt",
    ],
)

cc_test(
    name = "batch_stream_test",
    size = "small",
    srcs = ["batch_stream_test.cc"],
    copts = ["-fno-access-control"],
    data = [
        "//modules/perception/common/inference:inference_test_data",
    ],
    deps = [
        ":batch_stream",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_package()
cpplint()
